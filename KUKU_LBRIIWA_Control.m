%
%
% Template to visualize in V-REP the inverse kinematics algorithm developed
%          for a 7-DOF robot of the family Kuka LWR or IIWA
%
% Read Instructions.odt first !
%
% Do not modify any part of this file except the strings within
%    the symbols << >>
%
% G. Antonelli, Sistemi Robotici, fall 2014


function [t, q, q_act] = main

    porta = 19997;          % default V-REP port
    tf = 10;                 % final time
    Ts = 0.01;              % sampling time
    t  = 0:Ts:tf;           % time vector
    N  = length(t);         % number of points of the simulation
    n = 7;                  % joint number
    q  = zeros(n,N);        % q(:,i) collects the joint position for t(i)
    dq  = zeros(n,N);       % q(:,i) collects the joint position for t(i)
    q(:,1) = [0 35  20 -45 30 30 -20]'/180*pi;   % initial configuration

    % <<
    %
    % Put here any initialization code: DH table, gains, final position,
    % cruise velocity, etc.
    %
    % >>

    clc
    fprintf('----------------------');
    fprintf('\n simulation started ');
    fprintf('\n trying to connect...\n');
    [clientID, vrep ] = StartVrep(porta);

    handle_joint = my_get_handle_Joint(vrep,clientID);      % handle to the joints
    my_set_joint_target_position(vrep, clientID, handle_joint, q(:,1)); % first move to q0
    q_act(:,1) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep

    % main simulation loop
    for i=2:N
        % next line to be commented when running his own code
        dq(:,i) = 1*sin((2*pi/2)*t(i));

        % <<
        %
        % Put here :
        %   computation of the desired trajectory in the operative space
        %   computation of the error
        %   computation of the desired joint velocity: dq(:,i)
        %
        % >>

        q(:,i) = q(:,i-1) + Ts*dq(:,i);
        my_set_joint_target_position(vrep, clientID, handle_joint, q(:,i));
        q_act(:,i) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
    end

    DeleteVrep(clientID, vrep);

end

% constructor
function [clientID, vrep ] = StartVrep(porta)

    vrep = remApi('remoteApi');   % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1);        % just in case, close all opened connections
    clientID = vrep.simxStart('192.168.0.100',porta,true,true,5000,5);% start the simulation

    if (clientID>-1)
        disp('remote API server connected successfully');
    else
        disp('failed connecting to remote API server');
        DeleteVrep(clientID, vrep); %call the destructor!
    end
    % to change the simulation step time use this command below, a custom dt in v-rep must be selected,
    % and run matlab before v-rep otherwise it will not be changed
    % vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, 0.002, vrep.simx_opmode_oneshot_wait;

end

% destructor
function DeleteVrep(clientID, vrep)

    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
%   vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
    vrep.simxFinish(clientID);  % close the line if still open
    vrep.delete();              % call the destructor!
    disp('simulation ended');

end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)

    [m,n] = size(q);
    for i=1:n
        for j=1:m
            err = vrep.simxSetJointTargetPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
            if (err ~= vrep.simx_error_noerror)
                fprintf('failed to send joint angle q %d \n',j);
            end
        end
    end

end

function handle_joint = my_get_handle_Joint(vrep,clientID)

    [~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint1',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint2',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint3',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint4',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint5',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint6',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)

    [~,n] = size(q);

    for i=1:n
        joints_positions = vrep.simxPackFloats(q(:,i)');
        [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);

        if (err~=vrep.simx_return_ok)
           fprintf('failed to send the string signal of iteration %d \n',i);
        end
    end
    pause(8);% wait till the script receives all data, increase it if dt is too small or tf is too high

end


function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)

    for j=1:n
         vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
    end

    pause(0.05);

    for j=1:n
         [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
    end

    if (err(j)~=vrep.simx_return_ok)
           fprintf(' failed to get position of joint %d \n',j);
    end

end