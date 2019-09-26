function R = rpytoR(a)
    R1 = [cos(a(1)) sin(a(1)) 0;-sin(a(1)) cos(a(1)) 0;0 0 1];
    R2 = [cos(a(2)) 0 -sin(a(2));0 1 0;sin(a(2)) 0 cos(a(2))];
    R3 = [1 0 0;0 cos(a(3)) sin(a(3));0 -sin(a(3)) cos(a(3))];
    R = R3*R2*R1;
end