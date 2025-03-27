FUEL_UNEVEN = 1.0

function onTick()
    local leftFuel = input.getNumber(1)
    local rightFuel = input.getNumber(2)
    local pumpIn = input.getBool(1) and not input.getBool(2)
    local pumpOut = input.getBool(2) and not input.getBool(1)
    local neitherPump = not (pumpIn or pumpOut)
    local leftLow = (rightFuel - leftFuel) > FUEL_UNEVEN
    local rightLow = (leftFuel - rightFuel) > FUEL_UNEVEN

    local leftPump = (neitherPump and leftLow) or (pumpIn and not rightLow)
    local leftValve = rightLow or (pumpOut and not leftLow)
    local rightPump = (neitherPump and rightLow) or (pumpIn and not leftLow)
    local rightValve = leftLow or (pumpOut and not rightLow)

    output.setBool(1, pumpOut)
    output.setBool(2, pumpIn)
    output.setBool(3, leftPump)
    output.setBool(4, leftValve)
    output.setBool(5, rightPump)
    output.setBool(6, rightValve)
end