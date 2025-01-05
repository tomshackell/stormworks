require("Tom.PID")

IB_RUN = 1
IN_TARGET_RPS = 1
IN_COMBUST_RPS = 2
IN_ENGINE_STARTED_RPS = 3
IN_STARTER_THROTTLE = 4
IN_PID_P = 5
IN_PID_I = 6
IN_PID_D = 7
IN_PID_BIAS = 8
IN_PID_I_DECAY = 9
IN_PID_MIN_OUTPUT = 10
IN_PID_MAX_OUTPUT = 11

OB_COMPRESSOR_ON = 1
ON_STARTER_MOTOR = 1
ON_JET_THROTTLE = 2

local jetPID = nil

function onTick()
    local run = input.getBool(IB_RUN)
    local targetRPS = input.getNumber(IN_TARGET_RPS)
    local combustRPS = input.getNumber(IN_COMBUST_RPS)    
    local engineStartedRPS = input.getNumber(IN_ENGINE_STARTED_RPS)
    local starterThrottle = input.getNumber(IN_STARTER_THROTTLE)

    local running = combustRPS > engineStartedRPS
    local starterMotor = run and not running and 1.0 or 0.0
    local jetThrottle = 0
    if run then
        if running then
            if not jetPID then
                jetPID = PID:new()
            end
            jetPID.kp = input.getNumber(IN_PID_P)
            jetPID.ki = input.getNumber(IN_PID_I)
            jetPID.kd = input.getNumber(IN_PID_D)
            jetPID.bias = input.getNumber(IN_PID_BIAS)
            jetPID.idecay = input.getNumber(IN_PID_I_DECAY)
            jetPID.minOut = input.getNumber(IN_PID_MIN_OUTPUT)
            jetPID.maxOut = input.getNumber(IN_PID_MAX_OUTPUT)

            jetThrottle = jetPID:update(targetRPS, combustRPS)
        else
            jetThrottle = starterThrottle
            jetPID = nil
        end
    else
        jetPID = nil
    end
    output.setBool(OB_COMPRESSOR_ON, run)
    output.setNumber(ON_STARTER_MOTOR, starterMotor)
    output.setNumber(ON_JET_THROTTLE, jetThrottle)
end



