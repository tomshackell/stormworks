require("Tom.PID")
require("Tom.Math")
require("Tom.RingBuffer")

pid = PID:new({})
outputBuffer = ResizableRingBuffer:new(60)

display = { xTicks = 1.0, yMin = -100, yMax = 100, showOutput = false }

function onTick()
    local setpointSignal = input.getNumber(1) or 0
    local setpointGain = input.getNumber(2) or 1
    local measuredSignal = input.getNumber(3) or 0
    local measuredGain = input.getNumber(4) or 1
    local divisor = input.getNumber(10) or 1
    pid.kp = (input.getNumber(5) or 0) / divisor
    pid.ki = (input.getNumber(6) or 0) / divisor
    pid.kd = (input.getNumber(7) or 0) / divisor
    pid.minOut = input.getNumber(8) or -1
    pid.maxOut = input.getNumber(9) or 1
    
    display.xTicks = math.max(input.getNumber(11) or 1, 1)
    display.yMin = input.getNumber(12) or -1
    display.yMax = input.getNumber(13) or 1
    display.showOutput = input.getBool(1)

    local setpoint = setpointSignal * setpointGain
    local measured = measuredSignal * measuredGain
    local value = pid:update(setpoint, measured)
    outputBuffer:resize(math.ceil(display.xTicks))
    outputBuffer:push({ setpoint = setpoint, measured = measured, output = value })

    output.setNumber(1, value)
end

function onDraw()
    local w, h = screen.getWidth(), screen.getHeight()
    local last = nil
    local i = 0
    local yFor = function(v, min, max)
        return (min >= max) and 0 or lerp(v, min, h, max, 0)
    end
    local zeroY = yFor(0, display.yMin, display.yMax)
    screen.setColor(64, 64, 64)
    screen.drawLine(0, zeroY, w, zeroY)
    for v in outputBuffer:iter() do
        local x = i / display.xTicks * w
        local setpointY = yFor(v.setpoint, display.yMin, display.yMax)
        local measuredY = yFor(v.measured, display.yMin, display.yMax)
        local outputY = yFor(v.output, pid.minOut, pid.maxOut)
        if last then
            if display.showOutput then
                screen.setColor(0, 0, 255)
                screen.drawLine(last.x, last.outputY, x, outputY)
            end
            screen.setColor(255, 0, 0)
            screen.drawLine(last.x, last.setpointY, x, setpointY)
            screen.setColor(255, 255, 0)
            screen.drawLine(last.x, last.measuredY, x, measuredY)
        end
        last = { x = x, setpointY = setpointY, measuredY = measuredY, outputY = outputY }
        i = i + 1
    end
end
