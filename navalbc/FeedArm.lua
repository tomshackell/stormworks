require("Tom.PID")
require("Tom.Math")

ticks = 0

rotPID = PID:new({kp=2, minOut=-1, maxOut=1})

function onTick()
    local sourceLoaded = input.getBool(1)
    local feedArmLoaded = input.getBool(2)
    local drainLoaded = input.getBool(3)
    local rotIsVelocityPivot = input.getBool(4)

    local feedArmRotPosn = input.getNumber(1)
    local sourceTwist = input.getNumber(2)
    local drainTwist = input.getNumber(3)
    local sourceRot = input.getNumber(4)
    local drainRot = input.getNumber(5)
    local rotPIDKp = input.getNumber(6)

    local targetRot = 0
    local feedArmTwist = 0
    local sourceFeederMotor = false
    local drainFeederMotor = false

    if feedArmLoaded then
        -- move the arm to "drain" to off-load a shell
        targetRot = drainRot
        feedArmTwist = drainTwist
        drainFeederMotor = not drainLoaded
    else
        -- move the arm to "source" to pick-up a shell
        targetRot = sourceRot
        feedArmTwist = sourceTwist
        sourceFeederMotor = sourceLoaded
    end

    local feedArmRotMotor = 0
    if rotIsVelocityPivot then
        local delta = angularDiffTurns(feedArmRotPosn, targetRot)
        rotPID.kp = rotPIDKp
        feedArmRotMotor = rotPID:update(delta, 0)
    else
        -- NOTE: non-velocity pivots are 1=0.25 turns, convert from turns to motor setting
        feedArmRotMotor = targetRot * 4
    end

    output.setNumber(1, feedArmRotMotor)
    output.setNumber(2, feedArmTwist)
    output.setBool(1, sourceFeederMotor)
    output.setBool(2, drainFeederMotor)

    ticks = ticks + 1
end




