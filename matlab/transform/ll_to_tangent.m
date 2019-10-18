function p = ll_to_tangent(lat,long,lat0,long0,h0)
    Kflat = 1/298.257222101;
    Ra = 6378137.0+h0;%6372.1363*1e3;%
    Re = (1-Kflat.*sin(lat0).^2)*Ra;
    Rphi = Re*cos(lat0);
    A = Re*sin(lat-lat0);
    B = Rphi*(1-cos(long-long0))*sin(lat0);
    C = Rphi*sin(long-long0);
    p = [C, A+B];
end