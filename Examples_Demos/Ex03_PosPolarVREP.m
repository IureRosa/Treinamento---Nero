% Tarefa 01:
% Levar o robô da posição (0 m, 0 m) com orientaçaõ 0 graus,
% para a posição (2 m, 2 m) com orientação 0 graus
%
% Tarefa 02:
% Utilizar o comanda 'input' para permitir a entrada da posição e
% orientação desejada

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
V = VREP;
V.vConnect;
V.vHandle('Pioneer_p3dx');

%% Load and stop Pioneer
V.vHandle('Pioneer_p3dx');
Ud = [0; 0];
V.vSendControlSignals(Ud,1);

%% Collecting and plotting measurements
% Get Robot Position
[Pos.Xc,Pos.X, Pos.U] = V.vGetSensorData(1);

% P.rConnect; % Descomentar para realiazação de experimento
% P.rSetPose([0 0 0 0]);

% Tempo de esperar para início do experimento/simulação
dados = [];
pause(1)

%% Definindo a Figura que irá rodar a simulação
figure(1)
hold on
grid on
title('Simulação');
axis([-1 3 -1 3])

% Pontos desejados
Xd = [3 0 3; 2 1 0];
k = 1;
% Tempo Real
tmax = 30;
tc = tic;
tp = tic;
t = tic;

%% Inicio da simulação
while (toc(t) <= tmax)
    % Sub-rotina de controle
    if toc(tc) > 0.1
        % Inicio da realimentação
        tc = tic;
        
        if toc(t) > 10*k
            k = k + 1;
        else
            V.pPos.Xd(1:2) = Xd(:,k);
        end
            
        
        % Pegando os dados do robo
        vGetSensorData(P);
        
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        rho   = norm(P.pPos.Xtil(1:2));
        theta = atan2(P.pPos.Xtil(2),P.pPos.Xtil(1));
        alpha = theta - P.pPos.X(6);
        if abs(alpha) > pi
            if alpha > 0
                alpha =  2*pi-alpha;
            else
                alpha = -2*pi-alpha;
            end
        end
        ku = 0.75/(1+rho);
        ka = 2*pi/3/(1+abs(alpha));
        % -----------------------------------------------------
        % Criar o código aqui: Substituir os sinais de controle
        P.pSC.Ud(1) = ku*cos(alpha)*rho;
        P.pSC.Ud(2) = ku*cos(alpha)*sin(alpha) + ka*alpha;
        % -----------------------------------------------------
        
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
        
        Ud = [0.1; 0];
V.vSendControlSignals(Ud,1);
        % Enviar sinais de controle para o robô         
        P.rSendControlSignals;
        
        
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
P.rSendControlSignals;

