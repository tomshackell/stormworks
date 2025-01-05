---@section RingBuffer 
-- A class providing a ring buffer
RingBuffer = {
    -- create a new ring-buffer of the given max size
    new = function(cls, maxSize)
        return {
            items={},        -- the items stored in the ring buffer
            sz=maxSize + 1,  -- max size of the ring buffer
            writeIdx=0,      -- current index for writing
            readIdx=0,       -- current index for reading
            -- methods 
            push=cls.push,
            iter=cls.iter,
        }
    end,
    -- write a value to the ring buffer: wrapping if needed
    push = function(self, value)
        self.items[self.writeIdx] = value
        self.writeIdx = (self.writeIdx + 1) % self.sz
        if self.writeIdx == self.readIdx then
            self.readIdx = (self.readIdx + 1) % self.sz
        end
    end,
    -- iterate through the items in the ring-buffer, FIFO order
    iter = function(self, offset)
        local lastI, i = -1, (self.readIdx + (offset or 0)) % self.sz
        return function()
            lastI, i = i, (i + 1) % self.sz
            if lastI == self.writeIdx then
                return nil
            else
                return self.items[lastI]
            end
        end
    end
}
---@endsection

---@section ResizableRingBuffer 
--- A RingBuffer that supports being dynamically resized
ResizableRingBuffer = {
    new = function(cls, size)
        local obj = {
            buffer = {},
            capacity = size,
            start = 1,
            count = 0,
            push = cls.push,
            resize = cls.resize,
            iter = cls.iter,
        }
        obj.class = cls
        return obj
    end,

    push = function(self, item)
        if self.count < self.capacity then
            self.buffer[(self.start + self.count - 1) % self.capacity + 1] = item
            self.count = self.count + 1
        else
            -- Overwrite the oldest item when buffer is full
            self.buffer[self.start] = item
            self.start = (self.start % self.capacity) + 1
        end
    end,

    resize = function(self, new_size)
        if new_size ~= self.capacity then
            if new_size < self.count then
                -- Drop the oldest items to fit in the new size
                local drop_count = self.count - new_size
                self.start = (self.start + drop_count - 1) % self.capacity + 1
                self.count = new_size
            end

            -- Rebuild the buffer with the new capacity
            local new_buffer = {}
            for i = 1, self.count do
                new_buffer[i] = self.buffer[(self.start + i - 2) % self.capacity + 1]
            end

            self.buffer = new_buffer
            self.capacity = new_size
            self.start = 1
        end
    end,

    iter = function(self)
        local index = 0
        return function()
            if index < self.count then
                index = index + 1
                return self.buffer[(self.start + index - 2) % self.capacity + 1]
            end
        end
    end
}
---@endsection