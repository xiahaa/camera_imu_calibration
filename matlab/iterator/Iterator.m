classdef Iterator < handle
    methods(Abstract)
        hasNext(~);
        next(~);
    end
end