% Tarefa 1: Conecte um joystick XBOX e utilize as hastes anal�gicas para
% guiar o rob�

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
% P.rConnect; % Descomentar para realiaza��o de experimento
P.rSetPose([0 0 0 0]);

% Criar objeto Joystick XBOX
J = JoyControl;

% Tempo de esperar para in�cio do experimento/simula��o
pause(1)

%% Definindo a Figura que ir� rodar a simula��o
figure(1)
hold on
grid on
title('Simula��o');
axis([-1 3 -1 3])

%% Tempo Real
tmax = 30;
ta = tic;
tp = tic;
t = tic;

%% Inicio da simula��o
while (toc(t) <= tmax)
    if toc(ta) > 0.1 % Tempo de envio de comandos
        % Inicio da realimenta��o
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
        % Plot da simula��o
        P.mCADdel
        % P.mCADplot(1,'r') % Visualiza��o 3D
        P.mCADplot2D('r')   % Visualiza��o 2D
        drawnow
    end
end

P.pSC.Ud = [0; 0];
P.rSendControlSignals

