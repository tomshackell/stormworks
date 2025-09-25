require("Tom.PID")
require("Tom.Math")

EJECT_WAIT_TICKS = 25       -- how long to wait after firing before moving the ejector forward
EJECT_FWD_TICKS = 30
EJECT_BACK_TICKS = 32

EJECT_BEGIN_FWD = EJECT_WAIT_TICKS
EJECT_BEGIN_BACK = EJECT_BEGIN_FWD + EJECT_FWD_TICKS
EJECT_END_CYCLE = EJECT_BEGIN_BACK + EJECT_BACK_TICKS

BREACH_CLOSED = 0           -- breach is closed
BREACH_OPENING = 1          -- was closed, and is now opening
BREACH_OPEN = 2             -- breach fully open, ready to load a shell
BREACH_LOADING = 3          -- breach is loading a shell
BREACH_CLOSING = 4          -- breach was open, now closing with a shell loaded

BREACH_OPEN_TICKS = 60     -- how long does it take the breach to open
BREACH_LOAD_TICKS = 40     -- how long does it take to load a shell into the open breach

CLOSE_ENOUGH = 0.001       -- in turns, so is 0.36 degrees

ticks = 0
breachState = BREACH_CLOSED
breachTime = 0
ejectTime = -EJECT_END_CYCLE   -- start at the end of cycle

elevationPID = PID:new({ kp = 4, minOut = -1, maxOut = 1 })
rotationPID = PID:new({ kp = 20, minOut = -1, maxOut = 1 })

function onTick()
    -- input --------------------------------------------------------------------------------
    local currentElevation = input.getNumber(1) or 0
    local targetElevation = input.getNumber(2) or 0
    local currentRotation = input.getNumber(3) or 0
    local targetRotation = input.getNumber(4) or 0

    local breachLoaded = input.getBool(1) or false
    local ammoAvailable = input.getBool(2) or false
    local consentToFire = input.getBool(3) or false
    local powerOn = input.getBool(4) or false
    if not powerOn then return end

    -- elevation and rotation --------------------------------------------------------------    
    local elevationMotor = elevationPID:update(targetElevation, currentElevation)
    local rotationDelta = angularDiffTurns(currentRotation, targetRotation)
    local rotationMotor = rotationPID:update(rotationDelta, 0)

    -- breach open/close -------------------------------------------------------------------
    local openBreach = false
    local feedBreach = false
    if breachState == BREACH_CLOSED then
        if not breachLoaded then
            breachState = BREACH_OPENING
            breachTime = ticks + BREACH_OPEN_TICKS
        end
    elseif breachState == BREACH_OPENING then
        openBreach = true
        if ticks >= breachTime then
            ejectTime = ticks           -- begin ejection cycle
            breachState = BREACH_OPEN
        end
    elseif breachState == BREACH_OPEN then
        openBreach = true
        if ammoAvailable then
            breachState = BREACH_LOADING
            breachTime = ticks + BREACH_LOAD_TICKS
        end
    elseif breachState == BREACH_LOADING then
        openBreach = true
        feedBreach = true
        if ticks >= breachTime then
            breachState = BREACH_CLOSING
        end
    elseif breachState == BREACH_CLOSING then
        if breachLoaded then
            breachState = BREACH_CLOSED
        end
    end

    -- ejection ---------------------------------------------------------------------------
    local ejectorMotor = -1
    if ticks >= ejectTime + EJECT_END_CYCLE then
        ejectorMotor = -1   -- ejection cycle complete
    elseif ticks >= ejectTime + EJECT_BEGIN_BACK then
        ejectorMotor = -1   -- move ejector back
    elseif ticks >= ejectTime + EJECT_BEGIN_FWD then
        ejectorMotor = 1    -- move ejector forward            
    end

    -- weapon trigger ----------------------------------------------------------------------
    local aimedAtTarget = math.abs(targetElevation - currentElevation) < CLOSE_ENOUGH
        and math.abs(rotationDelta) < CLOSE_ENOUGH
    local weaponTrigger = breachState == BREACH_CLOSED and breachLoaded
        and consentToFire and aimedAtTarget

    -- output ------------------------------------------------------------------------------
    output.setNumber(1, elevationMotor)
    output.setNumber(2, rotationMotor)
    output.setNumber(3, ejectorMotor)
    output.setBool(1, weaponTrigger)
    output.setBool(2, openBreach)
    output.setBool(3, feedBreach)

    ticks = ticks + 1
end