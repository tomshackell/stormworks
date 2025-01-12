-- VectorMath ---------------------------------------------------------------------------------------------------

---@section vAdd
function vAdd(a, b) return { x=a.x+b.x, y=a.y+b.y, z=a.z+b.z } end
---@endsection

---@section vSub
function vSub(a, b) return { x=a.x-b.x, y=a.y-b.y, z=a.z-b.z } end
---@endsection

---@section vMul
function vMul(a, f) return { x=a.x*f, y=a.y*f, z=a.z*f } end
---@endsection

---@section vNeg
function vNeg(a) return { x=-a.x, y=-a.y, z=-a.z } end
---@endsection

---@section vCross
function vCross(a, b) return { x = a.y*b.z - a.z*b.y, y = a.z*b.x - a.x*b.z, z = a.x*b.y - a.y*b.x } end
---@endsection

---@section vDot
function vDot(a, b) return a.x*b.x + a.y*b.y + a.z*b.z end
---@endsection

---@section vLen
function vLen(v) return math.sqrt(vDot(v, v)) end
---@endsection

---@section vNorm
function vNorm(v) return vMul(v, 1/vLen(v)) end
---@endsection

---@section vToYawPitch
---@param v vector3 the vector to get yaw & pitch for
---@return number, number: yaw & pitch in radians
function vToYawPitch(v)
    local pitch = math.asin(vDot(v, {x=0,y=0,z=1}))
    local fw = vNorm({x=v.x,y=v.y,z=0})
    local yaw = math.acos(vDot(fw, {x=0,y=1,z=0}))
    if vDot(fw, {x=1,y=0,z=0}) < 0 then yaw = 2*math.pi - yaw end
    return yaw,pitch
end
---@endsection

---@section vFromPolar
---@param bearing number the polar bearing (radians) 0 = North, +ve = CW
---@param pitch number the polar pitch (radians) +ve = up
---@return vector3: the bearing & pitch expressed as a direction vector
function vFromPolar(bearing, pitch)
    local f = math.cos(pitch)
    return { x = math.sin(bearing) * f, y = math.cos(bearing) * f, z = math.sin(pitch) }
end
---@endsection

---@section vFromDistanceAzimuthElevation
---@param distance number distance (in meters)
---@param azimuth number azimuth angle (in radians)
---@param elevation number elevation angle (in radians)
---@return vector3: the vector representation
function vFromDistanceAzimuthElevation(distance, azimuth, elevation)
    local q = distance * math.cos(elevation)
    return {
        x = q * math.sin(azimuth),
        y = q * math.cos(azimuth),
        z = distance * math.sin(elevation)
    }
end
---@endsection

---@section vToDistanceAzimuthElevation
---@return number, number, number: distance, azimuth in radians, elevation in radians
function vToDistanceAzimuthElevation(v)
    local distance = vLen(v)
    local elevation = math.atan(v.z, math.sqrt(v.x^2 + v.y^2))
    local azimuth = math.atan(v.x, v.y)
    return distance, azimuth, elevation
end 
---@endsection

-- Quaternions ---------------------------------------------------------------------------------------------------

---@section qLocalToGlobalFromSW
-- Given the sensors onboard return a local-to-global rotation quaternion, i.e.
--
--      qMulV(qLocalToGlobalFromSW(rightTilt, forwardTilt, compass), v)
--
-- will translate v from local coordinates of the sensors to global map 
-- coordinates: +x = right, +y = forward, +z = up.
-- 
--      qInverse(qLocalToGlobalFromSW(rightTilt, forwardTilt, compass))
--
-- gives the global-to-local transformation that will rotate a global point into this local frame of reference.
--
---@param forward number the value from a tilt sensor facing "forward" (+ve y)
---@param right number the value from a tilt sensor facing "right" (+ve x)
---@param compass number the value from a Stormworks compass
---@return quaternion: the local-to-global orientation quaternion
function qLocalToGlobalFromSW(forward, right, compass)
    return qFromEuler(turnsToRad(forward), -turnsToRad(right), turnsToRad(compass))
end
---@endsection

---@section qFromEuler
function qFromEuler(roll, pitch, yaw)
    local cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    local cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    local cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return { w = cr * cp * cy + sr * sp * sy,
             x = sr * cp * cy - cr * sp * sy,
             y = cr * sp * cy + sr * cp * sy,
             z = cr * cp * sy - sr * sp * cy }
end
---@endsection

---@section qAddQ
function qAddQ(a, b)
    return { w=a.w+b.w, x=a.x+b.x, y=a.y+b.y, z=a.z+b.z }
end
---@endsection

---@section qSubQ
function qSubQ(a, b)
    return { w=a.w-b.w, x=a.x-b.x, y=a.y-b.y, z=a.z-b.z }
end
---@endsection

---@section qScale
function qScale(q, f)
    return { w=q.w*f, x=q.x*f, y=q.y*f, z=q.z*f }
end
---@endsection

---@section qMulQ
function qMulQ(a, b)
    return {
        w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    }
end 
---@endsection

---@section qMulV
function qMulV(q, v)
    local t = vMul(vCross(q, v), 2)
    return vAdd(vAdd(v, vMul(t, q.w)), vCross(q, t))
end
---@endsection

---@section qFromScalar
function qFromScalar(s) return { w=s,x=0,y=0,z=0 } end
---@endsection

---@section qInverse
function qInverse(q) 
    local i = 1.0 / (q.w^2 + q.x^2 + q.y^2 + q.z^2)
    return { w=q.w*i, x=-q.x*i, y=-q.y*i, z=-q.z*i }
end
---@endsection

---@section qConjugate
function qConjugate(q) 
    return { w=q.w, x=-q.x, y=-q.y, z=-q.z }
end
---@endsection

---@section qAngularVelocity 
-- calculates the 'body' angular velocity going from qfrom to qto in dt time
---@param qfrom quaternion the initial orientation
---@param qto quaternion the final orientation
---@param dt number the time taken to go from qfrom to qto (should be small)
---@return vector3: angular velocity vector {x=roll, y=pitch, z=yaw} radians per second
function qAngularVelocity(qfrom, qto, dt)
    -- taken from https://math.stackexchange.com/a/2313263
    local derivative = qScale(qSubQ(qto, qfrom), 1 / dt)
    local r = qMulQ(qConjugate(qScale(qfrom, 2)), derivative)
    return { x=r.x, y=r.y, z=r.z }
end
---@endsection


-- Stormworks specific ---------------------------------------------------------------------------------------------------

---@section compassToBearing
---@param compass number the stormworks compass input as read from the sensor
---@return number: normal bearing in degrees (0 = North, 90 = East, etc.)
function compassToBearing(compass) return (360 - compass*360) % 360 end
---@endsection

---@section cameraZoomToFOVRad
function cameraZoomToFOVRad(fov) return lerpClamp(fov, 0, 2.356194490192, 1, 0.025) end
---@endsection

---@section turnsToRad
function turnsToRad(t) return t*math.pi*2 end
---@endsection

-- General Math ----------------------------------------------------------------------------------------------------------

---@section radToDeg
function radToDeg(r) return r*180/math.pi end
---@endsection

---@section degToRad
function degToRad(d) return d*math.pi/180 end
---@endsection

---@section clamp
function clamp(x, minX, maxX) return math.min(math.max(x, minX), maxX) end
---@endsection

---@section lerp
function lerp(x, minX, minY, maxX, maxY) return (x - minX) / (maxX - minX) * (maxY - minY) + minY end
---@endsection

---@section lerpClamp
function lerpClamp(x, minX, minY, maxX, maxY) return clamp(lerp(x, minX, minY, maxX, maxY), minY, maxY) end
---@endsection

---@section nanGuard
function nanGuard(x, r) return (x == x) and x or (r or 0) end
---@endsection

--- Calculate the angular difference between two angles (in degrees), given the circular property 
--- of angles. For example if current=350 and target=10, then the angular difference is 20.
---@section angularDiffDeg
function angularDiffDeg(current, target)
    local diff = ((target % 360) - (current % 360) + 360) % 360
    return (diff > 180) and (diff - 360) or diff
end
---@endsection

-- Function to invert a 3x3 matrix
---@section invert3x3
function invert3x3(matrix)
    -- Extract elements of the matrix
    local a, b, c, d, e, f, g, h, i = unpack(matrix)

    -- Calculate the determinant of the 3x3 matrix
    local det = a * determinant2x2(e, f, h, i) - b * determinant2x2(d, f, g, i) + c * determinant2x2(d, e, g, h)

    -- Check if the matrix is singular (non-invertible)
    if det == 0 then
        return nil -- Return nil if the matrix is not invertible
    end

    -- Calculate the inverse matrix elements & return  the inverted matrix
    local invDet = 1 / det
    return {
        determinant2x2(e, f, h, i) / det, -determinant2x2(b, c, h, i) / det, determinant2x2(b, c, e, f) / det, 
        -determinant2x2(d, f, g, i) / det, determinant2x2(a, c, g, i) / det, -determinant2x2(a, c, d, f) / det, 
        determinant2x2(d, e, g, h) / det, -determinant2x2(a, b, g, h) / det, determinant2x2(a, b, d, e) / det
    }
end
---@endsection

-- Function to calculate the determinant of a 2x2 matrix
---@section determinant2x2
function determinant2x2(a, b, c, d)
    return a * d - b * c
end
---@endsection
