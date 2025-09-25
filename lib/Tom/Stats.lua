---@section incrementalStdDev
-- Welford's online algorithm for calculating std deviation
-- https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
function incrementalStdDev()
    local count, mean, m2 = 0, 0, 0
    return function(newValue)
        count = count + 1
        local delta = newValue - mean
        mean = mean + (delta / count)
        m2 = m2 + delta * (newValue - mean)
        return mean, math.sqrt(m2 / count)
    end
end
---@endSection

---@section fitGaussian
function fitGaussian(points)
    local n = #points
    if n == 0 then return nil end

    -- mean
    local mx, my, mz = 0, 0, 0
    for _, p in ipairs(points) do
        mx = mx + p.x    
        my = my + p.y
        mz = mz + p.z
    end
    mx, my, mz = mx/n, my/n, mz/n

    -- variance (isotropic)
    local var = 0
    for _, p in ipairs(points) do
        local dx, dy, dz = p.x - mx, p.y - my, p.z - mz 
        var = var + (dx*dx + dy*dy + dz*dz)
    end
    var = var / (2*n)   -- divide by 2 for 2D isotropic
    local sigma = math.sqrt(var)
    return { mean = {x=mx, y=my, z=mz}, sigma = sigma }
end
---@endSection