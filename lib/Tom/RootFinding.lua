---@section find1DRootUsingBisection
---@param f a function of a single argument, 'x', that returns some value 'y'.
---@param minX the minimum value of 'x' to search for
---@param maxX the maximum value of 'x' to search for
---@param epsilon if 'f(x)' is within 'epsilon' of 0 then this 'x' is deemed 'close enough'
---@return the 'x' such that 'f(x)' is within 'epsilon' of 0, or 'nil' if there is no solution between 'minX' and 'maxX'
function find1DRootUsingBisection(f, minX, maxX, epsilon)
    if not minX or not maxX then 
        return nil, 0 
    end 
    local minY, maxY = f(minX), f(maxX)
    if not minY or not maxY or minY * maxY > 0 then
        return nil, 0 -- no solution
    end
    local lowX, lowY, highX, highY = minX, minY, maxX, maxY
    local iters = 0
    while math.abs(highY - lowY) > epsilon do
        local midX = (lowX + highX) / 2
        local midY = f(midX)
        if midY * lowY > 0 then -- mid is same sign as low, so this refines low
            lowX, lowY = midX, midY
        else 
            highX, highY = midX, midY
        end
        iters = iters + 1
    end 
    return (lowX + highX) / 2, iters
end
---@endsection

---@section find1DRootUsingBrent converges quicker than using bisection, but requires more code
---@param f a function of a single argument, 'x', that returns some value 'y'.
---@param minX the minimum value of 'x' to search for
---@param maxX the maximum value of 'x' to search for
---@param epsilon if 'f(x)' is within 'epsilon' of 0 then this 'x' is deemed 'close enough'
---@return the 'x' such that 'f(x)' is within 'epsilon' of 0, or 'nil' if there is no solution between 'minX' and 'maxX'
function find1DRootUsingBrent(f, minX, maxX, epsilon)
    local a, fa, b, fb = minX, f(minX), maxX, f(maxX)
    if not fa or not fb or fa * fb > 0 then
        return nil, 0 -- no solution
    end
    local c, fc, s, fs = a, fa, b, fb
    local d = b - a
    local e = d
    local abs = math.abs
    local iters = 0
    while abs(b - a) > epsilon do
        if fa ~= fc and fb ~= fc then
            -- Inverse quadratic interpolation
            s = a * fb * fc / ((fa - fb) * (fa - fc))
              + b * fa * fc / ((fb - fa) * (fb - fc))
              + c * fa * fb / ((fc - fa) * (fc - fb))
        else 
            -- Secant
            s = b - fb * (b - a) / (fb - fa)
        end
        local cond = s < (3 * a + b) / 4 or s > b or
            abs(e) < epsilon or abs(s - b) >= abs(e / 2) or 
            (abs(e) >= epsilon and abs(s - b) >= abs(d / 2)) 
        if cond then 
            -- Bisection
            s, e = (a + b) / 2, b - a
            d = e
        else 
            d = e
            e = b - s
        end
        fs = f(s)
        if not fs then return nil end
        c, fc = b, fb
        if fa * fs < 0 then
            b, fb = s, fs
        else 
            a, fa = s, fs
        end
        if abs(fa) < abs(fb) then
            a, b = b, a 
            fa, fb = fb, fa
        end
        if abs(fb) < epsilon then 
            break
        end
        iters = iters + 1
    end 
    return b, iters
end
---@endsection

---@section find2DRootUsingNewton
---
--- uses Newton-Raphson to find the root of a 2D function using a jacobian. The jacobian should be defined as
--- a function that takes two arguments t, u. And produces a 2x2 jacobian, and two residuals f1 & f2.
---
--- The 2x2 jacobian should be:
--- { { d(f1)/dt, d(f1)/du },
---   { d(f2)/dt, d(f2)/du } }
---
--- @param jacobian function of t, u
--- @param t0 initial value of t
--- @param u0 initial value of u
--- @param epsilon (optional) stop when solution is close enough (in t and u) by this, or 1e-6 if not provided
--- @param maxIters (optional) maximum iterations before giving up, or 20 if not provided.
--- @return the resulting t, u if it converges, or nil otherwise
function find2DRootUsingNewton(jacobian, t0, u0, epsilon, maxIters)
    local t, u = t0, u0 
    local epsilon = epsilon or 1e-6
    for iter = 1, maxIters or 20 do 
        local J, f1, f2 = jacobian(t, u)
        local det = J[1][1] * J[2][2] - J[1][2] * J[2][1]
        if math.abs(det) < 1e-8 then break end -- singular Jacobian
        local invJ = { 
            {  J[2][2] / det, -J[1][2] / det },
            { -J[2][1] / det,  J[1][1] / det } 
        }
        local dt = invJ[1][1] * f1 + invJ[1][2] * f2
        local du = invJ[2][1] * f1 + invJ[2][2] * f2 
        t = t - dt
        u = u - du
        if math.abs(dt) < epsilon and math.abs(du) < epsilon then
            return t, u, iter
        end
    end
    return nil -- no convergence
end
---@endsection

---@section ternarySearch
---@param f a function of a single argument x 
---@param minX the minimum value of x to search from
---@param maxY the maximum value of x to search to 
---@param epsilon how close to the optimum x must be before we stop searching
---@return the value x between minX and maxX such that f(x) is maximized (within epsilon)
function ternarySearch(f, minX, maxX, epsilon)
    local loX, hiX = minX, maxX 
    while math.abs(hiX - loX) > epsilon do
        local third = (hiX - loX) / 3
        local x1, x2 = loX + third, hiX - third
        local y1, y2 = f(x1), f(x2)
        if y1 <= y2 then loX = x1 end
        if y1 >= y2 then hiX = x2 end
    end
    return (hiX + loX) / 2
end
---@endsection

