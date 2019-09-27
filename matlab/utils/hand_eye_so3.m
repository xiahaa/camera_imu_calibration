function varargout = hand_eye_so3(X,Y)
    if size(X,2) == 2
        X3 = cross(X(:,1),X(:,2));
        Y3 = cross(Y(:,1),Y(:,2));
        X = [X X3./norm(X3)];
        Y = [Y Y3./norm(Y3)];
        M = X*inv(Y);
        R = inv(sqrtm(M'*M))*M';
%         R = 
    else
        M = zeros(3,3);
        for i = 1:size(X,2)
            M = M + X(:,i)*Y(:,i)';
        end
        R = inv(sqrtm(M'*M))*M';
    end
    varargout{1} = R;
end


        
        