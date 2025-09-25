--- Developed using LifeBoatAPI - Stormworks Lua plugin for VSCode - https://code.visualstudio.com/download (search "Stormworks Lua with LifeboatAPI" extension)
--- If you have any issues, please report them here: https://github.com/nameouschangey/STORMWORKS_VSCodeExtension/issues - by Nameous Changey

require("Tom.PID")
require("Tom.Math")

local timeNow = 0

function newGun()
    -- Gun states 
    local EMPTY = 0                   -- gun is empty: wait for round to chamber 
    local READY = 1                   -- gun is ready to fire
    local RELOAD_POSITION = 2         -- reloading: wait for the gun to get in position
    local RELOAD_LOADING = 3          -- reloading: wait for the rounds to load 
    
    local INIT_TIME = 10              -- time for the system to settle before we start running
    local ELEVATION_EPSILON = 0.001   -- elevation is close enough (turns)
    local AZIMUTH_EPSILON = 0.001     -- azimuth is close enough (turns)
    local GUN_FULLY_LOADED = 76       -- this many rounds in the gun drums means it's fully loaded

    local chamberWasLoaded = false
    local state = EMPTY

    local elevatorPID = PID:new({ kp=2, ki=0, kd=0, minOut=-1, maxOut=1 })
    local azimuthPID = PID:new({ kp=20, ki=0, kd=0, minOut=-1, maxOut=1 })
    local carriageState = newCarriage()
    local ejectorState = newEjector()

    local tick = function(ins)
        local carriage = carriageState.tick({
            elevation = ins.currentCarriageElevation,
            azimuth = ins.currentCarriageAzimuth,
            gunAzimuth = ins.currentGunAzimuth,
            gunReadyToLoad = state == RELOAD_LOADING,
            gunFullyLoaded = ins.gunAmmo >= GUN_FULLY_LOADED,
            carriageAmmo = ins.carriageAmmo
        })
        local gun = {
            ejectorVelocity = 0,
            elevationMotor = 0,
            azimuthMotor = 0,
            chamberFeeder = false,
            feedSwitch = false,
            weaponTrigger = false,
        }

        local isAimedAt = function(tgtElevation, tgtAzimuth)
            return math.abs(tgtElevation - ins.currentGunElevation) <= ELEVATION_EPSILON and
                math.abs(tgtAzimuth - ins.currentGunAzimuth) <= AZIMUTH_EPSILON
        end

        local ammoAvailable = ins.leftAmmoAvailable or ins.rightAmmoAvailable or ins.junctionLoaded
        local targetAzimuth = ins.currentGunAzimuth
        local targetElevation = ins.currentGunElevation

        local turnsTo0 = angularDiffTurns(ins.currentGunAzimuth, 0)
        local turnsTo180 = angularDiffTurns(ins.currentGunAzimuth, 0.5)
        local reloadFwd = math.abs(turnsTo0) < math.abs(turnsTo180) or not ins.allowRearReload
        local reloadAzimuth = reloadFwd and 0 or 0.5

        if state == EMPTY then
            if timeNow > INIT_TIME and ins.feederLoaded then
                gun.weaponTrigger = not ins.chamberLoaded   -- flash trigger to load first round
                state = READY
            end
        elseif state == READY then
            local aimedAtTarget = isAimedAt(ins.targetGunElevation, ins.targetGunAzimuth)
            gun.weaponTrigger = aimedAtTarget and ins.consentToFire
            targetElevation = ins.targetGunElevation
            targetAzimuth = ins.targetGunAzimuth
            if not ammoAvailable and not ins.feederLoaded and not ins.chamberLoaded then
                state = RELOAD_POSITION -- turret must reload
            end
        elseif state == RELOAD_POSITION then
            targetElevation = 0
            targetAzimuth = reloadAzimuth
            if isAimedAt(0, targetAzimuth) then
                state = RELOAD_LOADING
            end
        elseif state == RELOAD_LOADING then
            targetElevation = 0
            targetAzimuth = reloadAzimuth
            if carriage.gunUnlock then
                state = EMPTY -- carriage is clear: go back to normal operations, after loading a round
            end
        end

        -- ejection -----------------------------------------------------------------------------
        local justFired = chamberWasLoaded and not ins.chamberLoaded
        chamberWasLoaded = ins.chamberLoaded
        gun.ejectorVelocity = ejectorState.tick(justFired)

        -- feeder control -----------------------------------------------------------------------
        gun.chamberFeeder = not ins.chamberLoaded and ammoAvailable
        gun.feedSwitch = not ins.leftAmmoAvailable and ins.rightAmmoAvailable

        -- gun elevation -----------------------------------------------------------------------
        gun.elevationMotor = elevatorPID:update(targetElevation, ins.currentGunElevation)

        -- gun azimuth --------------------------------------------------------------
        local deltaAzimuth = angularDiffTurns(ins.currentGunAzimuth, targetAzimuth)
        gun.azimuthMotor = azimuthPID:update(deltaAzimuth, 0)
        return gun, carriage
    end
    return { tick = tick }
end

function newEjector()
    local WAIT_TICKS = 15
    local FWD_TICKS = 16
    local BACK_TICKS = 18
    local FULL_CYCLE = WAIT_TICKS + FWD_TICKS + BACK_TICKS
    
    local fireTimes = {}

    local tick = function(justFired)
        if justFired then
            table.insert(fireTimes, timeNow)
        end
        local ejectorVelocity = 0
        while #fireTimes > 0 do
            local topTime = fireTimes[1]
            if timeNow >= topTime + FULL_CYCLE then
                -- cycle complete: pop from the stack
                table.remove(fireTimes, 1)
            elseif timeNow >= topTime + WAIT_TICKS + FWD_TICKS then
                ejectorVelocity = -1
                break   -- in the back phase
            elseif timeNow >= topTime + WAIT_TICKS then
                ejectorVelocity = 1
                break   -- in the forward phase
            else
                break   -- in the wait phase
            end
        end
        return ejectorVelocity
    end
    return { tick = tick }
end

function newCarriage()
    local DOCKED = 0                -- carriage is docked: ready to receive rounds
    local RAISE = 1                 -- load rounds from carriage into the gun until gun is full 
    local LOWER = 2                 -- lower the carriage down from the gun
    
    local LOWERED = 0.01            -- less than this and the carriage is fully lowered
    local CARRIAGE_FULL = 80        -- total rounds held by the carriage

    local state = LOWER

    local tick = function(ins)
        local result = {
            azimuthMotor = 0, elevationMotor = 0,
            feedToGun = false, feedToCarriage = false,
            gunUnlock = false,
        }
        if state == DOCKED then
            result.gunUnlock = false       -- gun can move as it likes
            result.elevationMotor = -1      -- keep carriage low (prevent it drifting up)
            if ins.gunReadyToLoad then
                state = RAISE               -- lift up carriage to fill gun
            end
            result.feedToCarriage = ins.carriageAmmo < CARRIAGE_FULL
        elseif state == RAISE then
            result.elevationMotor = 1       -- raise up to gun
            result.feedToGun = true         -- feed as soon as possible
            result.gunUnlock = false       -- don't move gun while filling
            if ins.gunFullyLoaded then
                state = LOWER               -- gun is full: lower carriage to start restocking it
            end
        elseif state == LOWER then
            result.elevationMotor = -1      -- lower the carriage
            result.gunUnlock = false       -- wait until carriage is lowered before gun can move
            if ins.elevation <= LOWERED then
                state = DOCKED
                result.gunUnlock = true
            end
        end
        return result
    end
    return { tick = tick }
end

function angularDiffTurns(current, target)
    return angularDiffDeg(current * 360, target * 360) / 360
end

local gunState = newGun()

function onTick()
    local ins = {
        targetGunElevation = input.getNumber(1), -- quarters (1=up,-1=down) to turns
        currentGunElevation = input.getNumber(2),
        targetGunAzimuth = input.getNumber(3),
        currentGunAzimuth = input.getNumber(4),
        currentCarriageElevation = input.getNumber(5),
        currentCarriageAzimuth = input.getNumber(6),
        carriageAmmo = input.getNumber(7),
        gunAmmo = input.getNumber(8),

        leftAmmoAvailable = input.getBool(1),
        rightAmmoAvailable = input.getBool(2),
        feederLoaded = input.getBool(3),
        chamberLoaded = input.getBool(4),
        junctionLoaded = input.getBool(5),
        consentToFire = input.getBool(6),
        allowRearReload = input.getBool(7),
    }
    local gun, carriage = gunState.tick(ins)

    output.setNumber(1, gun.ejectorVelocity)
    output.setNumber(2, gun.elevationMotor)
    output.setNumber(3, gun.azimuthMotor)
    output.setNumber(4, carriage.elevationMotor)
    output.setNumber(5, carriage.azimuthMotor)

    output.setBool(1, gun.chamberFeeder)
    output.setBool(2, gun.feedSwitch)
    output.setBool(3, gun.weaponTrigger)
    output.setBool(4, carriage.feedToGun)
    output.setBool(5, carriage.feedToCarriage)

    timeNow = timeNow + 1
end
