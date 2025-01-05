---@section PQueue
PQueue={
    new=function(cls) return { d={}, n=0, insert=cls.insert, pop=cls.pop } end,
    insert=function(self, v, p)
        self.d[self.n] = {v=v, p=p}
        self.n = self.n + 1
        local i = self.n - 1
        while i > 0 and self.d[parent(i)].p > self.d[i].p do
            swap(self, i, parent(i))
            i = parent(i)
        end
    end,
    pop=function(self)
        if self.n == 0 then return nil end
        self.n = self.n - 1
        local r = self.d[0].v
        self.d[0] = self.d[self.n]
        heapify(self, 0)
        return r
    end
}

function parent(i) return math.floor((i-1)/2) end

function heapify(h, i)
    if h.n < 1 then return end
    local a = 2*i+1
    local b = 2*i+2
    local z = (a < h.n and h.d[a].p < h.d[i].p) and a or i
    z = (b < h.n and h.d[b].p < h.d[z].p) and b or z
    if z ~= i then
        swap(h, i, z)
        heapify(h, z)
    end
end

function swap(h, i, j)
    local t = h.d[i]
    h.d[i] = h.d[j]
    h.d[j] = t
end
---@endsection

---@section __PQueue_test__
q = PQueue:new()
q:insert('A',4)
q:insert('B',3)
q:insert('C',6)
q:insert('D',2)
q:insert('E',5)
q:insert('F',1)
q:insert('G',9)
q:insert('H',8)
q:insert('I',7)
while true do
    v = q:pop()
    if v then print(v) else break end
end
---@endsection