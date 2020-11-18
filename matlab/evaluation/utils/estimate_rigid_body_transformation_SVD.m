function varargout = estimate_rigid_body_transformation_SVD(varargin)
%% SVD 
if nargin == 1
    ptsrc = varargin{1}(1:3,:);
    ptdst = varargin{1}(4:6,:);
else
    ptsrc = varargin{1};
    ptdst = varargin{2};
end

ptsrcmean = mean(ptsrc,2);
ptdstmean = mean(ptdst,2);

ptsrcrefine = ptsrc - repmat(ptsrcmean, 1, size(ptsrc,2));
ptdstrefine = ptdst - repmat(ptdstmean, 1, size(ptsrc,2));

Y = ptdstrefine';
X = ptsrcrefine;
S = X*Y;
[U,~,V] = svd(S);

D = V*U';
if det(D) < 0
    Ropt = V*[1 0 0;0 1 0;0 0 -1]*U';
else
    Ropt = V*U';
end
topt = ptdstmean - Ropt * ptsrcmean;

if nargout == 2
    varargout{1} = Ropt;
    varargout{2} = topt;
else
    res.Ropt = Ropt;
    res.topt = topt;
    varargout{1} = res;
end

end

