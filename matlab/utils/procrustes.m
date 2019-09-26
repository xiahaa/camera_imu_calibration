function varargout = procrustes(X,Y,remove_mean)
    if size(X,2) == 2
        X3 = cross(X(:,1),X(:,2));
        Y3 = cross(Y(:,1),Y(:,2));
        X = [X;X3./norm(X3)];
        Y = [Y;Y3./norm(X3)];
    end
    D = size(X,1);
    N = size(X,2);
    if remove_mean
        Xhat = X-mean(X,2);
        Yhat = Y-mean(Y,2);
    else
        Xhat = X;
        Yhat = Y;
    end
    [U,~,V]=svd(Xhat*Yhat');
    R = U*diag([1,1,det(U*V)])*V;
    varargout{1}=R;
    if remove_mean
        t = mean(X,2) - R*mean(Y,2);
        varargout{2}=t;
    else
        varargout{2}=[];
    end
end
