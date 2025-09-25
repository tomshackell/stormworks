---@section rmNew
function rmNew(m11, m12, m13, m21, m22, m23, m31, m32, m33)
    return { 
        { m11, m12, m13 }, 
        { m21, m22, m23 }, 
        { m31, m32, m33 } 
    }
end
---@endsection

---@section rmFromEuler
--- Creates a rotation matrix for local-to-global using the given euler angles
--- Convention: x-axis = right, y-axis = up, z-axis = forward, left-hand coordinate system
---@param ex rotation around the x-axis (pitch, +ve = down)
---@param ey rotation around the y-axis (yaw, +ve = right)
---@param ez rotation around the z-axis (roll, +ve = left)
function rmFromEuler(ex, ey, ez)
    local cx, sx = math.cos(ex), math.sin(ex)
    local cy, sy = math.cos(ey), math.sin(ey)
    local cz, sz = math.cos(ez), math.sin(ez)
    return {
        { cy*cz, sx*sy*cz-cx*sz, cx*sy*cz+sx*sz },
        { cy*sz, sx*sy*sz+cx*cz, cx*sy*sz-sx*cz },
        { -sy,   sx*cy,          cx*cy          }
    }
end
---@endsection

---@section rmBuild
function rmBuild(f)
    return { 
        { f(1, 1), f(1, 2), f(1, 3) },
        { f(2, 1), f(2, 2), f(2, 3) },
        { f(3, 1), f(3, 2), f(3, 3) }
    }
end
---@endsection

---@section rmTranspose
function rmTranspose(m)
    return rmBuild(function(i, j) return m[j][i] end)
end
---@endsection

---@section rmMulV
function rmMulV(m, v)
    return vBuild(function(i) return m[i][1]*v.x + m[i][2]*v.y + m[i][3]*v.z end)
end
---@endsection

---@section rmMulRM
function rmMulRM(a, b)
    return rmBuild(function(i, j) return a[i][1]*b[1][j] + a[i][2]*b[2][j] + a[i][3]*b[3][j] end)
end
---@endsection

---@section vBuild
function vBuild(f) return { x=f(1), y=f(2), z=f(3) } end 
---@endsection

---@section rmPrint
function rmPrint(m)
    print(""..m[1][1]..", "..m[1][2]..", "..m[1][3])
    print(""..m[2][1]..", "..m[2][2]..", "..m[2][3])
    print(""..m[3][1]..", "..m[3][2]..", "..m[3][3])
end
---@endsection