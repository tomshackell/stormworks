-- Vector -----------------------------------------------------------------------------------------

---@section vNew
vNew = table.pack
---@endsection

---@section vLenSq 
function vLenSq(v) return vDot(v, v) end
---@endsection

---@section vDot
function vDot(a, b)
    checkEq(a.n, b.n)
    local t = 0
    for i,av in ipairs(a) do t = t + av * b[i] end
    return t
end
---@endsection

---@section vLen
function vLen(v) return math.sqrt(vLenSq(v)) end
---@endsection

---@section vAdd
function vAdd(a, b, f)
    checkEq(a.n, b.n)
    return vBuild(a.n, 1, function(i) return a[i] + b[i] * (f or 1) end)
end
---@endsection

---@section vMul
function vMul(a, f)
    return vBuild(a.n, 1, function(i) return a[i] * f end)
end
---@endsection

---@section vBuild
function vBuild(n, sums, f)
    local r = {n=n}
    for i = 1, n do
        for k = 1, sums do
            r[i] = (r[i] or 0) + f(i, k)
        end
    end
    return r
end
---@endsection

---@section vToString
function vToString(v)
    local r = "["
    for i = 1, v.n do
        if i ~= 1 then r = r .. ", " end
        r = r .. tostring(v[i])
    end
    return r .. "]"
end
---@endsection

-- Matrix -----------------------------------------------------------------------------------------

---@section mNew
function mNew(rows, cols, ...)
    local r, input = { rows=rows, cols=cols }, {...}
    checkEq(rows * cols, #input)
    for i = 1, rows do
        r[i] = vBuild(cols, 1, function(j) return input[(i-1)*cols + j] end)
    end
    return r
end
---@endsection

---@section mAdd
function mAdd(a, b, f)
    checkEq(a.rows, b.rows, a.cols, b.cols)
    return mBuild(a.rows, a.cols, 1, function(i, j) return a[i][j] + b[i][j] * (f or 1) end)
end
---@endsection

---@section mMulM
function mMulM(a, b)
    checkEq(a.cols, b.rows)
    return mBuild(a.rows, b.cols, a.cols, function(i, j, k) return a[i][k] * b[k][j] end)
end
---@endsection

---@section mMulV
function mMulV(a, b)
    checkEq(a.cols, b.n)
    return vBuild(a.rows, a.cols, function(i, j) return a[i][j] * b[j] end)
end
---@endsection

---@section mTranspose
function mTranspose(m)
    return mBuild(m.cols, m.rows, 1, function(i, j) return m[j][i] end)
end
---@endsection

---@section mInvert3x3
function mInvert3x3(m)
    checkEq(m.rows, 3, m.cols, 3)
    local a, b, c = table.unpack(m[1])
    local d, e, f = table.unpack(m[2])
    local g, h, i = table.unpack(m[3])

    -- Calculate the determinant of the 3x3 matrix
    local A, B, C = e * i - h * f, f * g - d * i, d * h - e * g
    local det = a * A + b * B + c * C

    -- Check if the matrix is singular (non-invertible)
    if det == 0 then return nil end
    return mNew(3, 3,
        A / det, (c * h - b * i) / det, (b * f - c * e) / det,
        B / det, (a * i - c * g) / det, (d * c - a * f) / det,
        C / det, (g * b - a * h) / det, (a * e - d * b) / det
    )
end
---@endsection

---@section mBuild
function mBuild(rows, cols, sums, f)
    local r = { rows=rows, cols=cols }
    for i = 1, rows do
        r[i] = vBuild(cols, sums, function(j, k) return f(i, j, k) end)
    end
    return r
end
---@endsection

---@section mToString
function mToString(m)
    local r = "[" .. m.rows .. "x" .. m.cols .. "| "
    for i = 1, m.rows do
        if i ~= 1 then r = r .. " | " end
        for j = 1, m.cols do
            if j ~= 1 then r = r .. ", " end
            r = r .. m[i][j]
        end
    end
    return r .. " ]"
end
---@endsection

---@section checkEq
function checkEq(x1, x2, y1, y2)
    if x1 ~= x2 or y1 ~= y2 then error() end
end
---@endsection

-- Test -------------------------------------------------------------------------------------------

---@section Matrix_tests
function Matrix_tests()
    local x = vNew(1, 2, 3)
    local y = vNew(4, 5, 6)
    assertVEq(x, "[1, 2, 3]", "vNew & vToString")
    assertEq(vLenSq(x), 14, "vLenSq")
    assertEq(vLen(vNew(0, 4, 3)), 5, "vLen")
    assertEq(vDot(x, y), 32, "vDot")
    assertVEq(vAdd(x, y), "[5, 7, 9]", "vAdd")
    assertVEq(vMul(x, 3), "[3, 6, 9]", "vMul")

    local m1 = mNew(3, 3, 1, 2, 3, 4, 5, 6, 7, 8, 9)
    local m2 = mNew(3, 3, 10, 20, 30, 40, 50, 60, 70, 80, 90)
    local m3 = mNew(3, 3, 0, -3, -2, 1, -4, -2, -3, 4, 1)
    assertMEq(m1, "[3x3| 1, 2, 3 | 4, 5, 6 | 7, 8, 9 ]", "mNew & mToString")
    assertMEq(mAdd(m1, m2), "[3x3| 11, 22, 33 | 44, 55, 66 | 77, 88, 99 ]", "mAdd")
    assertMEq(mMulM(m1, m2), "[3x3| 300, 360, 420 | 660, 810, 960 | 1020, 1260, 1500 ]", "mMulM")
    assertVEq(mMulV(m1, x), "[14, 32, 50]", "mMulV")
    assertMEq(mTranspose(m1), "[3x3| 1, 4, 7 | 2, 5, 8 | 3, 6, 9 ]", "mTranspose")
    assertMEq(mInvert3x3(m3), "[3x3| 4.0, -5.0, -2.0 | 5.0, -6.0, -2.0 | -8.0, 9.0, 3.0 ]", "mInvert3x3")
end

function assertEq(a, b, message)
    assert(a == b, message .. " got " .. a .. ", expected " .. b)
end
function assertVEq(v, str, message)
    local got = vToString(v)
    assert(got == str, message .. " got '" .. got .. "', expected '" .. str .. "'")
end
function assertMEq(m, str, message)
    local got = mToString(m)
    assert(got == str, message .. " got '" .. got .. "', expected '" .. str .. "'")
end
function assert(c, msg)
    if not c then print(msg) end
end
---@endsection


