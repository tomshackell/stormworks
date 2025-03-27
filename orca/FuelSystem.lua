FUEL_UNEVEN = 1.0

function onTick()
    local leftFuel = input.getNumber(1)
    local rightFuel = input.getNumber(2)
    local leftPumpIn, leftPumpOut = input.getBool(1), input.getBool(2)
    local rightPumpIn, rightPumpOut = input.getBool(3), input.getBool(4)

    local pumpIn = (leftPumpIn or rightPumpIn) and not (leftPumpOut or rightPumpOut)
    local pumpOut = (leftPumpOut or rightPumpOut) and not (leftPumpIn or rightPumpIn)
    local leftLow = (rightFuel - leftFuel) > FUEL_UNEVEN
    local rightLow = (leftFuel - rightFuel) > FUEL_UNEVEN

    --------------------+------------------------------
    --  PI  PO  LL  RL  |   P   LI  LO  RI  RO
    --------------------+------------------------------
    --                  |
    --              T   |   T   T           T
    --          T       |   T       T   T
    --      T           |   T   T       T
    --      T       T   |   T   T
    --      T   T       |   T           T
    --  T               |   T       T       T
    --  T           T   |   T   T           T
    --  T       T       |   T       T   T

    local pumpOnOff = pumpIn or pumpOut or leftLow or rightLow
    local leftInValve = rightLow or (pumpOut and not leftLow)
    local leftOutValve = (leftLow and not pumpOut) or (pumpIn and not rightLow)
    local rightInValve = leftLow or (pumpOut and not rightLow)
    local rightOutValve = (rightLow and not pumpOut) or (pumpIn and not leftLow)

    output.setNumber(1, leftFuel + rightFuel)
    output.setBool(1, pumpIn or pumpOut)
    output.setBool(2, pumpOnOff)
    output.setBool(3, pumpIn)
    output.setBool(4, pumpOut)
    output.setBool(5, leftInValve)
    output.setBool(6, leftOutValve)
    output.setBool(7, rightInValve)
    output.setBool(8, rightOutValve)
end