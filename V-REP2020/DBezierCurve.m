function [Curve] = DBezierCurve(ControlPoints,time)
    % Number of control points on bezier curve
    N = size(ControlPoints,2);
    if nargin==1
        t=0:0.01:1;  % time
    else
        t = time;
    end
    
    Sum = 0;
    for i=0:N-1
        Sum = Sum + nchoosek(N-1,i)*(1-t).^(N-1-i).*(t.^i).*ControlPoints(:,i+1);
    end
    Curve = Sum;
    % Sum = 0;
    % for k=0:N-2
    % %        - Derivada da curva de B�zier:
    %     Sum = Sum + nchoosek(N-1,k).*ControlPoints(:,k+1).*((1-t).^(N-2-k)).*(t.^(k-1)).*(-t.*(N-1)+k);
    % end
    % dCurve = Sum;
end