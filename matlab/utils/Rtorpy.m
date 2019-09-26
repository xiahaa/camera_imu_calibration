function a = Rtorpy(R)
    a(1) = atan2(R(1,2),R(1,1));
    a(2) = -asin(R(1,3));
    a(3) = atan2(R(2,3),R(3,3));
end