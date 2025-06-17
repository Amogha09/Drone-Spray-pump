--[[ GENERAL SCRIPT VARIABLES ]] --
local MAV_SEVERITY_LEVEL = 6
local INTERVAL_MS = 3
local ACK_TIMEOUT = 10000
local prev_mode = nil
local prev_arm_status = false
local RTL_MODE = 6  -- RTL mode ID in ArduPilot

--[[ PARAMETERS ]] --
-- Command parameters for spraying modes
local SPRAY_CMD = {
    MANUAL = 0, -- 0 = Stop all spraying operations, 1 = Allow manual pump controls
    SPOT = 21,
    BLANKET = 22
}

local MANUAL_CMD = {
    PAUSE = 0,
    PAUSE_WITH_CONTROL = 1,
    RESUME = 2,
    RESTART = 3
}

-- Spray state for tracking
local spray_state = {
    state_ID = nil,       -- Ongoing mission ID
    cmd_type = nil,       -- SPRAY (1) / BLANKET (2)
    start_time = nil,     -- Time tracking for timeout
    received_ack = false, -- Acknowledgement flag
}

-- Custom mission states
local MISSION_STATUS = {
    INACTIVE = 0,
    ACTIVE = 1,
    PAUSED = 2,
    RESUMING = 3,
    COMPLETED = 4,
    RESTARTING = 5
}

-- Mission tracking
local mission_info = {
    status = MISSION_STATUS.INACTIVE,
    status_innate = mission:state(),
    seq = mission:get_current_nav_index(),
    reached_seq = mission:get_current_nav_index(),
}

-- Flowmeter data
local flowmeter_data = {
    flowrate = 0,
    volume = 0
}

-- [[ COPTER MODES ]] --
local ALT_HOLD_MODE = 2
local AUTO_MODE = 3
local LOITER_MODE = 5

-- [[ MAV NAV COMMANDS ]] --
local NAV_WAYPOINT = 16
local NAV_TAKEOFF = 22

--[[ MAVLINK SETUP ]]                                --
local mavlink_msgs = require("MAVLink/mavlink_msgs") -- Import MAVLink message handling functions (mainly for decoding)
local MAV_CHANNEL = 2                                -- Communicate in MAV_CHANNEL_2

-- Obtain MAVLink IDs
local NAMED_VALUE_FLOAT_ID = mavlink_msgs.get_msgid("NAMED_VALUE_FLOAT")
local COMMAND_ACK_ID = mavlink_msgs.get_msgid("COMMAND_ACK")

-- Assign MAVLink IDs to message map for decoding
local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[NAMED_VALUE_FLOAT_ID] = "NAMED_VALUE_FLOAT"

-- Initiate MAVLink with two message IDs (to look for)
mavlink:init(2, 10)
mavlink:register_rx_msgid(COMMAND_ACK_ID)
mavlink:register_rx_msgid(NAMED_VALUE_FLOAT_ID)

--[[ FUNCTIONS ]] --
-- Sends spot and blanket parameters to the companion computer
-- SPOT value = volume to spray
-- Blanket value = ON / OFF
-- v2.0: Modified send_command function to handle multiple arguments for blanket spraying
local function send_command(spray_cmd, value1, value2)
    local name
    local packed_value

    if spray_cmd == SPRAY_CMD.SPOT then
        name = "SPOT"
        packed_value = value1 -- Volume for spot spray
    elseif spray_cmd == SPRAY_CMD.BLANKET then
        name = "BLANKET"
        -- Multiply L/m by 1000 to preserve 3 decimal places
        -- Example: 1.234 L/m becomes 1234
        local lpm_packed = math.floor(value1 * 1000)
        -- Pack into single float: upper bits for L/m, lower bit for ON/OFF
        packed_value = (lpm_packed << 16) | value2
    elseif spray_cmd == SPRAY_CMD.MANUAL then
        name = "MANUAL"
        packed_value = value1 -- Determines ability for manual pump controls
    end

    local payload = string.pack('Ifz', millis():toint(), packed_value, name)
    mavlink:send_chan(MAV_CHANNEL, NAMED_VALUE_FLOAT_ID, payload)
end

-- Reset spray state to default
local function reset_spray_state()
    spray_state.state_ID = nil
    spray_state.cmd_type = nil
    spray_state.start_time = nil
    spray_state.received_ack = false
end

-- Handle acknowledgement message
local function handle_ack(msg)
    if msg.msgid == COMMAND_ACK_ID then
        if msg.command == spray_state.cmd_type then
            spray_state.received_ack = true
            spray_state.result = msg.result  -- Capture result type (ACCEPTED or FAILED)
        end
    end
end

-- Check for any incoming acknowledgement messages
local function check_ack()
    if spray_state.state_ID then
        if spray_state.received_ack then
            local ack_delay = spray_state.ack_time and (spray_state.ack_time - spray_state.start_time) or (millis() - spray_state.start_time)
            if ack_delay > 3500 then
                local action = param:get("SCR_USER1")
                if action == 1 then
                 --   gcs:send_text(MAV_SEVERITY_LEVEL, "SPRAY FAILED! Action: RTL")
                    vehicle:nav_script_time_done(spray_state.state_ID)
                    vehicle:set_mode(6)  -- RTL
                elseif action == 2 then
                  --  gcs:send_text(MAV_SEVERITY_LEVEL, "SPRAY FAILED! Action: Loiter")
                    vehicle:nav_script_time_done(spray_state.state_ID)
                    vehicle:set_mode(5)  -- Loiter
                elseif action == 3 then
                 --   gcs:send_text(MAV_SEVERITY_LEVEL, "SPRAY FAILED! Action: Land and Disarm")
                    vehicle:nav_script_time_done(spray_state.state_ID)
                    vehicle:set_mode(9)  -- Land
                    arming:disarm()
                else
                 --   gcs:send_text(MAV_SEVERITY_LEVEL, "SPRAY FAILED! Action: Next Waypoint")
                    vehicle:nav_script_time_done(spray_state.state_ID)
                end
            else
              --  gcs:send_text(MAV_SEVERITY_LEVEL, "SPRAY SUCCESS - Moving to next waypoint.")
                vehicle:nav_script_time_done(spray_state.state_ID)
            end
            reset_spray_state()

        elseif (millis() - spray_state.start_time) > ACK_TIMEOUT then
            gcs:send_text(MAV_SEVERITY_LEVEL, "TIMEOUT! SPRAY FAILED!")

            local action = param:get("SCR_USER1")
            if action == 1 then
                gcs:send_text(MAV_SEVERITY_LEVEL, "Timeout Action: RTL")
                vehicle:nav_script_time_done(spray_state.state_ID)
                vehicle:set_mode(6)  -- RTL
            elseif action == 2 then
                gcs:send_text(MAV_SEVERITY_LEVEL, "Timeout Action: Loiter")
                vehicle:nav_script_time_done(spray_state.state_ID)
                vehicle:set_mode(5)  -- Loiter
            elseif action == 3 then
                gcs:send_text(MAV_SEVERITY_LEVEL, "Timeout Action: Land and Disarm")
                vehicle:nav_script_time_done(spray_state.state_ID)
                vehicle:set_mode(9)  -- Land
                arming:disarm()
            else
                gcs:send_text(MAV_SEVERITY_LEVEL, "Timeout Action: Next Waypoint")
                vehicle:nav_script_time_done(spray_state.state_ID)
            end

            reset_spray_state()
        end
    end
    
    -- Receive message from MAVLink Channel
    local msg = mavlink:receive_chan()
    if msg then
       -- gcs:send_text(MAV_SEVERITY_LEVEL, "Message Received!")
        local decoded = mavlink_msgs.decode(msg, msg_map) -- Decode the message based on the registered IDs
        if decoded then
         --   gcs:send_text(MAV_SEVERITY_LEVEL, "Message Decoded!")
            handle_ack(decoded)
        end
    end
end

-- Retrieve flowmeter_data
local function check_flow()
    -- Receive message from MAVLink Channel
    local msg = mavlink:receive_chan()
    if msg then
        local decoded = mavlink_msgs.decode(msg, msg_map) -- Decode the message based on the registered IDs
        if decoded and decoded.msgid == NAMED_VALUE_FLOAT_ID then
            if decoded.name == "FLOWRATE" then
                flowmeter_data.flowrate = decoded.value
            elseif decoded.name == "VOLUME" then
                flowmeter_data.volume = decoded.value
            end
          --  gcs:send_named_float("flowrate", flowmeter_data.flowrate)
          --  gcs:send_named_float("volume", flowmeter_data.volume)
            logger:write("FLOW", "Flowrate,Volume", "II", flowmeter_data.flowrate, flowmeter_data.volume)

         --   gcs:send_text(MAV_SEVERITY_LEVEL, tostring(decoded.name) .. ": " .. tostring(decoded.value))
        end
    end
end


-- Retrieve mission status
local function check_mission_status()

    local current_status = mission:state()
    local current_mission_seq = mission:get_current_nav_index()

    -- Check mission state
    if mission_info.status_innate ~= current_status then
        if current_status == mission.MISSION_COMPLETE then -- Mission Complete
            mission_info.status = MISSION_STATUS.COMPLETED
          --  gcs:send_text(MAV_SEVERITY_LEVEL, "Mission: Mission Completed")

        elseif current_status == mission.MISSION_STOPPED then -- Mission Paused 
            mission_info.status = MISSION_STATUS.PAUSED
          --  gcs:send_text(MAV_SEVERITY_LEVEL, "Mission: Mission Paused")

        elseif current_status == mission.MISSION_RUNNING then -- Mission Resumed
            if mission_info.status_innate == mission.MISSION_STOPPED then
                mission_info.status = MISSION_STATUS.RESUMING
              --  gcs:send_text(MAV_SEVERITY_LEVEL, "Mission: Mission Resuming")
                send_command(SPRAY_CMD.MANUAL, MANUAL_CMD.RESUME)
                spray_state.start_time = millis()

            else -- New mission started 
                mission_info.status = MISSION_STATUS.ACTIVE
              --  gcs:send_text(MAV_SEVERITY_LEVEL, "Mission: Mission Active")
            end
        end
        mission_info.status_innate = current_status
    end

    if mission_info.status == MISSION_STATUS.RESTARTING then -- Check if drone has passed the first waypoint
        if mission:get_prev_nav_cmd_id() == NAV_WAYPOINT then
            mission_info.status = MISSION_STATUS.ACTIVE
           -- gcs:send_text(MAV_SEVERITY_LEVEL, "Mission: Mission Active")
        end

    elseif mission_info.status == MISSION_STATUS.RESUMING then
        if mission:get_prev_nav_cmd_id() == NAV_TAKEOFF then -- Check if is restart
            mission_info.status = MISSION_STATUS.RESTARTING
            mission_info.reached_seq = 0
            send_command(SPRAY_CMD.MANUAL, MANUAL_CMD.RESTART)
          --  gcs:send_text(MAV_SEVERITY_LEVEL, "Mission: Mission Restarted")

        elseif current_mission_seq > mission_info.reached_seq then -- Check if drone has reached the breakpoint
            mission_info.status = MISSION_STATUS.ACTIVE
          --  gcs:send_text(MAV_SEVERITY_LEVEL, "Mission: Mission Active")
        end
    end

    -- Check current mission index
    if mission_info.seq ~= current_mission_seq then

        if mission_info.status == MISSION_STATUS.ACTIVE or
            mission_info.status == MISSION_STATUS.COMPLETED then
            mission_info.reached_seq = mission_info.seq
        end

        if current_mission_seq < mission_info.seq and
            (mission:get_prev_nav_cmd_id() == NAV_TAKEOFF or
             mission:get_current_nav_id() == NAV_TAKEOFF) then
                mission_info.status = MISSION_STATUS.RESTARTING
                mission_info.reached_seq = 0
                send_command(SPRAY_CMD.MANUAL, MANUAL_CMD.RESTART)
              --  gcs:send_text(MAV_SEVERITY_LEVEL, "Mission: Mission Restarted")
        end
        mission_info.seq = current_mission_seq
    end
end

local function update()
    local id, cmd, arg1, arg2 = vehicle:nav_script_time()

    if id and id ~= spray_state.state_ID then
        if mission_info.status == MISSION_STATUS.ACTIVE then
            spray_state.state_ID = id
            spray_state.cmd_type = cmd
            spray_state.start_time = millis()
            spray_state.received_ack = false

            if cmd == SPRAY_CMD.SPOT then
                gcs:send_text(MAV_SEVERITY_LEVEL, string.format("Spot spray: %d mL", arg1))
                send_command(SPRAY_CMD.SPOT, arg1)
            elseif cmd == SPRAY_CMD.BLANKET then
                gcs:send_text(MAV_SEVERITY_LEVEL,
                    string.format("Blanket spray: %.4f L/m, State: %d", arg1, arg2))
                send_command(SPRAY_CMD.BLANKET, arg1, arg2)
            end
        else
            vehicle:nav_script_time_done(id)
        end

    end
    check_ack()
end

-- Fail-safe check
local function watchdog()
    local current_mode = vehicle:get_mode()
    local current_arm_status = arming:is_armed()
    check_mission_status()
    check_flow()

    -- Early exit conditions with TERMINATE command
    if prev_arm_status ~= current_arm_status and (not current_arm_status) then
        gcs:send_text(MAV_SEVERITY_LEVEL, "VEHICLE IS DISARMED!")
        prev_mode = current_mode
        prev_arm_status = current_arm_status
        return watchdog, INTERVAL_MS
    end

    -- Handle different modes
    if prev_mode ~= current_mode then
      --  gcs:send_text(MAV_SEVERITY_LEVEL, "Mode Change Detected!")

        if current_mode ~= AUTO_MODE then
            local allow_manual = (current_mode == ALT_HOLD_MODE or current_mode == LOITER_MODE) and MANUAL_CMD.PAUSE_WITH_CONTROL or MANUAL_CMD.PAUSE
            send_command(SPRAY_CMD.MANUAL, allow_manual)
            gcs:send_text(6, "MANUAL CONTROL " .. (allow_manual == MANUAL_CMD.PAUSE_WITH_CONTROL and "" or "NOT ") .. "ALLOWED")
        end
    end

    if current_mode == AUTO_MODE then
        update()
    end

    prev_mode = current_mode
    prev_arm_status = current_arm_status
    return watchdog, INTERVAL_MS
end

return watchdog()

