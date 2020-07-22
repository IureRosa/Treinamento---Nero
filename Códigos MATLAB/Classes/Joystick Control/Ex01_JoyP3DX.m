% Tarefa 1: Conecte um joystick XBOX e utilize as hastes analógicas para
% guiar o robô

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

%% Definindo o Robô
P = Pioneer3DX;
% P.rConnect; % Descomentar para realiazação de experimento
P.rSetPose([0 0 0 0]);

% Criar objeto Joystick XBOX
J = JoyControl;

% Tempo de esperar para início do experimento/simulação
pause(1)

%% Definindo a Figura que irá rodar a simulação
figure(1)
hold on
grid on
title('Simulação');
axis([-1 3 -1 3])

%% Tempo Real
tmax = 30;
ta = tic;
tp = tic;
t = tic;

%% Inicio da simulação
while (toc(t) <= tmax)
    if toc(ta) > 0.1 % Tempo de envio de comandos
        % Inicio da realimentação
        ta = tic;
        
        % Pegando os dados do robo
        P.rGetSensorData;
                
        % Controlador Joystick
        P = J.mControl(P);
        
        P.rSendControlSignals
    end
    
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

P.pSC.Ud = [0; 0];
P.rSendControlSignals

