---@section MovingAverage 
MovingAverage = {
    new = function(cls) 
        return { 
            ringBuffer={}, -- The ring buffer we use for calculation
            insertPoint=0, -- Where to insert next item in ring buffer
            bufferLength=0, -- number of valid values in the ring buffer
            totalWeightedValues=0, -- Running total of ring buffer: update after each value
            totalWeights=0,
            update=cls.update
        } 
    end,
    update = function(self, newValue, maxBufferSize, weight)
        local weight = weight or 1

        -- adjust size of buffer if required
    	if maxBufferSize <= self.bufferLength then
    		for i = maxBufferSize, self.bufferLength - 1 do
                local old = self.ringBuffer[i]
    			self.totalWeightedValues = self.totalWeightedValues - (old.value * old.weight)
                self.totalWeights = self.totalWeights - old.weight
    			self.ringBuffer[i] = nil
    		end
    		self.insertPoint = maxBufferSize > 0 and self.insertPoint % maxBufferSize or 0
    	end
    	
    	-- add new value into buffer and update totals
        local old = self.ringBuffer[self.insertPoint]
    	local oldValue = old and (old.value * old.weight) or 0
    	self.ringBuffer[self.insertPoint] = {value=newValue, weight=weight}
    	self.totalWeightedValues = self.totalWeightedValues + (newValue * weight) - oldValue
        self.totalWeights = self.totalWeights + weight - (old and old.weight or 0)
    	self.insertPoint = self.insertPoint + 1
    	if self.insertPoint >= maxBufferSize then
    		self.insertPoint = 0
    	end
    	return self.totalWeights > 0 and self.totalWeightedValues / self.totalWeights or 0
    end
}
---@endsection
