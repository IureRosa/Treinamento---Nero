% Positioning task

clear
close all
clc

try
    fclose(instrfindall);
catch
end
%% Carregando os objetos do cenário
V = VREP;
V.vConnect
V.vHandle('Pioneer_p3dx');

%% Definindo o Robô
P = Pioneer3DX;
P.pPar.a = 0;
P.pPar.alpha = -0.5*pi/4;
P.rConnect; % Descomentar para realiazação de experimento
% P.rSetPose
P.rSetPose([0 0 0 0]); 


%% Definindo a Figura que irá rodar a simulação
figure(1)
hold on
grid on
title('Simulação');
axis([-3 3 -3 3])
P.mCADplot2D('r')   % Visualização 2D
drawnow

% Tempo de esperar para início do experimento/simulação
dados = [];
pause(1)

%% Tempo Real
tmax = 60;
tc = tic;
tp = tic;
t = tic;

%% Inicio da simulação
while (toc(t) <= tmax)
    % Sub-rotina de controle
    if toc(tc) > 0.1
        % Inicio da realimentação
        tc = tic;
        
        P.pPos.Xda = P.pPos.Xd;
        if toc(t) > 30
            P.pPos.Xd(1:2) = [0; 0];
        else
            P.pPos.Xd(1:2) = [2; 2];
        end
        % Pegando os dados do robo
%         P.rGetSensorData;
        V.vGetSensorData(P,1);
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        % Controlador Cinemático modelo extendido
        vx = P.pPos.Xd(7) + 0.3*P.pPos.Xtil(1);
        vy = P.pPos.Xd(8) + 0.3*P.pPos.Xtil(2);
        
        % Para a*cos(alpha) ~= 0
        if abs(P.pPar.alpha) < pi/2 && P.pPar.a > 0
            vw = (-sin(P.pPos.X(6))*vx + cos(P.pPos.X(6))*vy)/(P.pPar.a*cos(P.pPar.alpha));
            P.pSC.Ud(2) = vw;
        else              
            P.pPos.Xd(6)  = atan2(vy,vx);
            P.pPos.Xd(12) = (P.pPos.Xd(6)-P.pPos.Xda(6))/0.1;            
            P.pPos.Xtil(6) = P.pPos.Xd(6) - P.pPos.X(6);
            vw = 0*(P.pPos.Xd(12)) + 1*P.pPos.Xtil(6);
        end
        
        P.pSC.Ud(2) = vw;
        P.pSC.Ud(1) = vx*cos(P.pPos.X(6)) + vy*sin(P.pPos.X(6)) + P.pPar.a*sin(P.pPar.alpha)*vw;

        % Armazenar dados da simulação
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];

        % Enviar sinais de controle para o robô
%         P.rSendControlSignals;    
         V.vSendControlSignals(P,1);
        
        % Sub-rotina para plotar
        if toc(tp) > 0.1
            tp = tic;
            % Plot da simulação
            P.mCADdel
            % P.mCADplot(1,'r') % Visualização 3D
            P.mCADplot2D('r')   % Visualização 2D
            drawnow
        end
        
    end
end


P.pSC.Ud([1 2]) = 0;
V.vSendControlSignals(P,1);

%% Desconecta Matlab e V-REP
V.vDisconnect;


%% Plot Charts

% Angular and Linear Velocity
figure
plot(dados(:,29),dados(:,25)) 
hold on
plot(dados(:,29),dados(:,26))
