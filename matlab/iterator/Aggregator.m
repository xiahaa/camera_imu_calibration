classdef Aggregator < handle
    methods(Abstract)
        iterObj = createIterator(~);
    end
end