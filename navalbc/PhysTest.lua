--require("Tom.Math")
require("Tom.PhysicsSensor")

function onTick()
    local tgt = { x=input.getNumber(20), y=input.getNumber(22), z=input.getNumber(21) }
    local phys = readPhysicsSensor()
    local r = phys:globalPointToLocal(tgt)

    output.setNumber(1, r.x)
    output.setNumber(2, r.y)
    output.setNumber(3, r.z)
end

