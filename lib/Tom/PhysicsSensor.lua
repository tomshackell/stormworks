require("Tom.Math")
require("Tom.RotMatrix")


---@section readPhysicsSensor
function readPhysicsSensor()
    local pitch, yaw, roll = input.getNumber(4), input.getNumber(5), input.getNumber(6)
    local localToGlobal = rmFromEuler(pitch, yaw, roll)
    local globalToLocal = rmTranspose(localToGlobal)   
    return {
        posn = { x=input.getNumber(1), y=input.getNumber(2), z=input.getNumber(3) },
        angles = { roll=roll, pitch=pitch, yaw=yaw },
        rmLocalToGlobal = localToGlobal,
        rmGlobalToLocal = globalToLocal,

        localVelocity = { x=input.getNumber(7), y=input.getNumber(8), z=input.getNumber(9) }, -- local linear speed, x:right+, y:up+, z:fwd+
        --angularVelocity = { x=-ang[3]*pi2, y=-ang[1]*pi2, z=ang[2]*pi2 }, -- local angular speed: x=roll, y=pitch, z=yaw
        absoluteSpeed = input.getNumber(13),
        compass = input.getNumber(17),

        -- Convert a position in global space (map coords: East=+x, North=+y, Up=+z) to local space (Right=+x, Forward=+y, Up=+z)
        globalPointToLocal = function(self, p)
            return rmMulV(self.rmGlobalToLocal, vSub(p, self.posn))
        end,
        -- Convert a point in local space (Right=+x, Forward=+y, Up=+z) to global space (map coords: East=+x, North=+y, Up=+z)
        localPointToGlobal = function(self, p)
            return vAdd(rmMulV(self.rmLocalToGlobal, p), self.posn)
        end,
    }
	--[[
	--tiltx = cthe/pi2
	--tilty = math.asin(-math.sin(cphi)*math.cos(cthe))/pi2
	tiltz = math.asin(-math.cos(cphi)*math.cos(cthe))/pi2
	
	sn(16,input.getNumber(15))--tilt x
	sn(17,input.getNumber(16))--tilt y
	sn(18,tiltz)--tilt z, for compatibility
	
	vew = rotate(AMS, vel)
	sn(19, vew[1])
	sn(20, -vew[2])
	sn(21, -vew[3])
	--19~21: world linear speed, X:East +, Y:North +, Z:upward +
    --]]
	
end
---@endSection