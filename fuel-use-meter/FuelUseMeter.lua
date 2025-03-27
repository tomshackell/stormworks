require("Tom.MovingAverage")
require("Tom.Math")

TICKS_PER_SECOND = 60
INIT_TICKS = 120

movAvg = MovingAverage:new()
lastFuelLevel = 0
ticks = 0

function onTick()
    local fuelLevel = input.getNumber(1)
    local bufferLength = round(input.getNumber(2) * TICKS_PER_SECOND)
    local result = 0

    if ticks > INIT_TICKS then
        local df = (lastFuelLevel - fuelLevel) * TICKS_PER_SECOND * 60
        result = movAvg:update(df, bufferLength)
    end
    lastFuelLevel = fuelLevel
    output.setNumber(1, result)
    ticks = ticks + 1
end
