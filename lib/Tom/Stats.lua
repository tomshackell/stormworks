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

---@section calc3DCovariance
--- Calculate the 3x3 covariance matrix for a collection of 3D points {x=..,y=..,z=..}
function calc3DCovariance(points)
    local n = #points
    if n < 2 then
        -- Covariance is not meaningful with less than 2 points
        return nil
    end

    -- Step 1: Calculate the mean for each dimension
    local sumX, sumY, sumZ = 0, 0, 0
    for _, p in ipairs(points) do
        sumX = sumX + p.x
        sumY = sumY + p.y
        sumZ = sumZ + p.z
    end

    local meanX = sumX / n
    local meanY = sumY / n
    local meanZ = sumZ / n

    -- Step 2: Calculate the covariance matrix components
    local covXX, covYY, covZZ = 0, 0, 0
    local covXY, covXZ, covYZ = 0, 0, 0

    for _, p in ipairs(points) do
        local dx = p.x - meanX
        local dy = p.y - meanY
        local dz = p.z - meanZ

        covXX = covXX + dx * dx
        covYY = covYY + dy * dy
        covZZ = covZZ + dz * dz

        covXY = covXY + dx * dy
        covXZ = covXZ + dx * dz
        covYZ = covYZ + dy * dz
    end

    -- Use n-1 for sample covariance
    local divisor = n - 1

    covXX = covXX / divisor
    covYY = covYY / divisor
    covZZ = covZZ / divisor

    covXY = covXY / divisor
    covXZ = covXZ / divisor
    covYZ = covYZ / divisor

    -- Assemble the final covariance matrix (3x3 table)
    local covarianceMatrix = {
        covXX, covXY, covXZ,
        covXY, covYY, covYZ, -- cov(y,x) is the same as cov(x,y)
        covXZ, covYZ, covZZ  -- cov(z,x) is the same as cov(x,z), etc.
    }

    return { mean={x=meanX, y=meanY, z=meanZ}, Rmeas = covarianceMatrix, samples = n }
end
---@endSection