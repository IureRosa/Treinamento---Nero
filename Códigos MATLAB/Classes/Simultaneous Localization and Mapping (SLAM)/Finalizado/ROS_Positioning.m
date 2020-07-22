% Positioning task

clear
close all
clc

try
    fclose(instrfindall);
end

try
    % Load Classes
    
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
%     % Inicializando o OptiTrack
    OPT = OptiTrack;    % Criando o OptiTrack
    OPT.Initialize;     % Iniciando o OptiTrack
    
    P = RPioneer(1,'P1'); % P�oneer3DX Experimento
    idP = 1;
    P.rDisableMotors;
    
    % Joystick
    J = JoyControl;
    
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end   

%% Definindo o Rob�
P = RPioneer(1,'P1');
P.pPar.a = 0;
P.pPar.alpha = -0.5*pi/4;
%% Definindo a Figura que ir� rodar a simula��o
figure(1)
hold on
grid on
title('Simula��o');
axis([-3 3 -3 3])
P.mCADplot2D('r')   % Visualiza��o 2D
drawnow

% Tempo de esperar para in�cio do experimento/simula��o
dados = [];
pause(1)
P.rEnableMotors;
%% Tempo Real
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
        
        P.pPos.Xda     = P.pPos.Xd;
        if toc(t) > 15
            P.pPos.Xd(1:2) = [0.4105; 1.4724];
        else
            P.pPos.Xd(1:2) = [1.5160; -2.1254];
        end
        % Pegando os dados do robo
%         P.rGetSensorData;
        rb = OPT.RigidBody;             % read optitrack
        
        try
            if rb(idP).isTracked
                P = getOptData(rb(idP),P);
%                 disp(P.pPos.X)
            end
        catch
        end
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
       P = fKinematicControllerExtended(P);

       P = J.mControl(P);
        
       P.rCommand;

        % Armazenar dados da simula��o
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
          
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

% P.pSC.Ud([1 2]) = 0;
P.rCommand;

J.pFlag = 0;
P.rDisableMotors;


