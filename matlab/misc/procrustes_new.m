function varargout = procrustes_new(X,Y,remove_mean)
    N = size(X,2);
    if size(X,2) == 2
        so31 = Y(1:3,1);
        so32 = X(1:3,1);
        so33 = Y(1:3,2);
        so34 = Y(1:3,2);

        M = [so31,so33,cross(so31,so33)]*inv([so32,so34,cross(so32,so34)]);
        R = inv(sqrtm(M'*M))*M';
    else
        M = zeros(3,3);
        for i = 1:N
            %% log
            so31 = X(1:3,i);
            so32 = Y(1:3,i);
            M = M + so31*so32';
        end
        R = inv(sqrtm(M'*M))*M';
    end
    varargout{1}=R;
end
