require("Tom.Math")
require("Tom.PID")
require("Tom.MovingAverage")

STOICHIOMETRIC = 0.2
AFR_SENSITIVITY = 0.05
STARTER_SUPER_BLEED = 0.1
RUNNING_RPS = 5
RESTART_RPS = 3
MIN_THROTTLE = 0.03
STARTER_THROTTLE = 0.1
TICKS_PER_SECOND = 60

rpsPID = PID:new({ kp = 0.1, ki = 0.001, minOut = MIN_THROTTLE, maxOut = 1 })
isRunning = false
afrDelta = 0

-- See https://www.reddit.com/r/Stormworks/comments/xdgmn2/tips_for_modular_engines/
function onTick()
    local engineOn = input.getBool(1)
    local cylinderAirVol = input.getNumber(1)
    local cylinderFuelVol = input.getNumber(2)
    local cylinderTemp = input.getNumber(3)
    local rps = input.getNumber(4)
    local inThrottle = clamp(input.getNumber(5), 0, 1)
    local minRPS = input.getNumber(6)
    local maxRPS = input.getNumber(7)
    rpsPID.kp = input.getNumber(8)
    rpsPID.ki = input.getNumber(9)
    rpsPID.kd = input.getNumber(10)
    local numCylinders = input.getNumber(11)

    if rps >= RUNNING_RPS then
        isRunning = true
    elseif rps <= RESTART_RPS then
        isRunning = false
    end

    local outThrottle, targetRPS, outAir, outFuel = 0, 0, 0, 0
    local stoichiometric, cylinderAFR = 0, 0
    local starting = engineOn and not isRunning
    if engineOn then
        targetRPS = lerp(inThrottle, 0, minRPS, 1, maxRPS)
        if starting then
            outThrottle = STARTER_THROTTLE
        else
            outThrottle = rpsPID:update(targetRPS, rps)
        end

        -- Calculate the AFR to set air and fuel throttles
        local t = clamp(cylinderTemp, 0, 100) * 0.01
        stoichiometric = lerp(t, 0, 14 - STOICHIOMETRIC*2, 1, 15 - STOICHIOMETRIC*5)
        cylinderAFR = cylinderFuelVol == 0 and 0 or (cylinderAirVol * 1000) / (cylinderFuelVol * 1000)
        afrDelta = clamp(afrDelta + ((1 - (stoichiometric / cylinderAFR)) * AFR_SENSITIVITY), -1, 1)
        outAir = outThrottle * clamp(1 - afrDelta, 0.1, 1)
        outFuel = outThrottle * clamp(1 + afrDelta, 0.1, 1)
    end

    -- decide whether to run the starter
    local starterThrottle = starting and 1 or 0

    -- we use a bleed valve to stop the supercharger from overpressuring and stalling.
    local superBleed = 0.5 - outThrottle * 0.1

    -- calculate fuel use rate
    local fuelUseRate = cylinderFuelVol * numCylinders * TICKS_PER_SECOND * 60

    output.setNumber(1, outAir)
    output.setNumber(2, outFuel)
    output.setNumber(3, superBleed)
    output.setNumber(4, starterThrottle)
    output.setNumber(5, targetRPS)
    output.setNumber(6, cylinderTemp)
    output.setNumber(7, stoichiometric)
    output.setNumber(8, cylinderAFR)
    output.setNumber(9, fuelUseRate)
end


