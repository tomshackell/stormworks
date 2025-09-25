EJECT_FWD_TICKS = 15
EJECT_BACK_TICKS = EJECT_FWD_TICKS + 16
EJECT_TICKS = EJECT_BACK_TICKS + 18

EMPTY = 0
CHAMBER = 1
READY = 2
RELOAD = 3

INIT_TIME = 10      -- time for the system to settle before we start running

local timeNow = 0
local chamberWasLoaded = false
local ejectTime = EJECT_TICKS
local state = EMPTY

function onTick()
    local targetElevation = input.getNumber(1)
    local currentElevation = input.getNumber(2)
    local targetAzimuth = input.getNumber(3)
    local currentAzimuth = input.getNumber(4)
    local currentCarriageElevation = input.getNumber(5)
    local currentCarriageAzimuth = input.getNumber(6)

    local leftAmmoAvailable = input.getBool(1)
    local rightAmmoAvailable = input.getBool(2)
    local feederLoaded = input.getBool(3)
    local chamberLoaded = input.getBool(4)
    local junctionLoaded = input.getBool(5)
    local consentToFire = input.getBool(6)

    local justFired = chamberWasLoaded and not chamberLoaded
    local ammoAvailable = leftAmmoAvailable or rightAmmoAvailable or junctionLoaded
    local weaponTrigger = false
    local elevationMotor = 0
    local azimuthMotor = 0
    local carriageElevationMotor = 0
    local carriageAzimuthMotor = 0

    if state == EMPTY then
        if timeNow > INIT_TIME and feederLoaded then
            weaponTrigger = not chamberLoaded   -- flash trigger to load first round
            state = READY
        end
    elseif state == READY then
        weaponTrigger = consentToFire
        if not ammoAvailable and not feederLoaded and not chamberLoaded then
            state = RELOAD -- turret must reload
        end
    elseif state == RELOAD then
        
    end

    -- ejection -----------------------------------------------------------------------------
    if ejectTime >= EJECT_TICKS and justFired then -- gun just fired
        ejectTime = 0 -- math.min(ejectTime - EJECT_TICKS, EJECT_FWD_TICKS)
    end
    local ejectorVelocity = 0
    if (ejectTime >= EJECT_FWD_TICKS and ejectTime < EJECT_BACK_TICKS) then
        ejectorVelocity = 1      -- move forward
    elseif ejectTime < EJECT_TICKS then
        ejectorVelocity = -1     -- move back
    end
    ejectTime = ejectTime + 1
    chamberWasLoaded = chamberLoaded

    -- feeder control -----------------------------------------------------------------------
    local chamberFeeder = not chamberLoaded and ammoAvailable
    local feedSwitch = not leftAmmoAvailable and rightAmmoAvailable

    -- output -------------------------------------------------------------------------------
    output.setNumber(1, ejectorVelocity)
    output.setNumber(2, elevationMotor)
    output.setNumber(3, azimuthMotor)
    output.setNumber(4, carriageElevationMotor)
    output.setNUmber(5, carriageAzimuthMotor)

    output.setBool(1, chamberFeeder)
    output.setBool(2, feedSwitch)
    output.setBool(3, weaponTrigger)

    timeNow = timeNow + 1
end

