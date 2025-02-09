function [p0,n0,Bmb,Nb,Vb,Ub,Curva] = CorrigeCurva1(A,B1,Curva)
%CorrigeCurva Summary of this function goes here
%   Detailed explanation goes here

    Curva.X =[];
    Curva.dX =[];
%     Curva.Vmax = Curva.Vmax;
%     n0 = A.pPos.X(6:8)';
%     p0 = A.pPos.X(1:3)'; z1 = 0.8
    p0 = [0,0.7,1.17]; n0 = [-1,0,0];% ponto inicial 
    
    % Orienta��o:
    Rz1 = [cos(B1.pPos.X(6)) -sin(B1.pPos.X(6)) 0;...
           sin(B1.pPos.X(6)) cos(B1.pPos.X(6)) 0; 0 0 1];

    
%     Rz3 = [cos(B3.pPos.X(6)) -sin(B3.pPos.X(6)) 0;...
%         sin(B3.pPos.X(6)) cos(B3.pPos.X(6)) 0; 0 0 1];

    Rn = [cos((90)*(pi/180)) -sin((90)*(pi/180)) 0;...
          sin((90)*(pi/180)) cos((90)*(pi/180)) 0; 0 0 1];


    % Bambol� 1:
    b1 = B1.pPos.X(1:3)'; 
    n1 = Rz1*[1;0;0]; n1 = Rn*n1; 
    v1 = B1.pPos.X(1:3)+[0;0.63;0]; v1 = v1./norm(v1); 
    u1 = B1.pPos.X(1:3)+[0;0;.63];  u1 = u1./norm(u1);
    
%     b3 = B3.pPos.X(1:3)'; 
%     n3 = Rz3*[1;0;0]; n3 = Rn*n3; 
%     v3 = B3.pPos.X(1:3)+[0;0.63;0]; v3 = v3./norm(v3); 
%     u3 = B3.pPos.X(1:3)+[0;0;.63];  u3 = u3./norm(u3);
    
    Bmb = [b1];
    Nb = [n1'];
    Ub = [u1']; Vb = [v1'];
    
%     Bmb = [b2;b1]
%     Nb = [n2';-n1']
%     Ub = [u2';u1']; Vb = [v2';v1'];
    
end

