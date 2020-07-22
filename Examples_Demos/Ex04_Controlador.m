clear
close all
clc

try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA ELT334';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Definindo o Rob�
P = Pioneer3DX;
P.pPar.a = 0.0015;
P.pPar.alpha = pi/180*0;

% P.rConnect; % Descomentar para realiaza��o de experimento
% P.rSetPose([0 0 0 0]);

% Tempo de esperar para in�cio do experimento/simula��o
dados = [];
pause(1)

%% Definindo a Figura que ir� rodar a simula��o
figure(1)
hold on
grid on
title('Simula��o');
axis([-1 3 -1 3])

% Pontos desejados
Xd = [3 0 3; 2 1 0];
k = 1;
% Tempo Real
tmax = 30;
tc = tic;
tp = tic;
t = tic;

%% Inicio da simula��o
while (toc(t) <= tmax)
    % Sub-rotina de controle
    if toc(tc) > 0.1
        % Inicio da realimenta��o
        tc = tic;
        
        % Posi��o desejada
        P.pPos.Xd(1) = 2;
        P.pPos.Xd(2) = 1;
        
        % Obter dados dos sensores
        P.rGetSensorData;
        
        % Calcular controlador
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        vx = P.pPos.Xd(7) + 0.5*P.pPos.Xtil(1);
        vy = P.pPos.Xd(8) + 0.5*P.pPos.Xtil(2);
        
        % Calcular velocidade angular
        P.pSC.Ud(2) = (-sin(P.pPos.X(6))*vx + cos(P.pPos.X(6))*vy)/(P.pPar.a*cos(P.pPar.alpha));
        P.pSC.Ud(1) = vx*cos(P.pPos.X(6)) + vy*sin(P.pPos.X(6)) + P.pSC.Ud(2)*P.pPar.a*sin(P.pPar.alpha); 
        
        % -----------------------------------------------------
        
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
        
        % Enviar sinais de controle para o rob�
        P.rSendControlSignals;
        
        
        % Sub-rotina para plotar
        if toc(tp) > 0.1
            tp = tic;
            % Plot da simula��o
            P.mCADdel
            % P.mCADplot(1,'r') % Visualiza��o 3D
            P.mCADplot2D('r')   % Visualiza��o 2D
            drawnow
        end
    end
end

P.pSC.Ud([1 2]) = 0;
P.rSendControlSignals;

