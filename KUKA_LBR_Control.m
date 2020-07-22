lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.80];
show(lbr);
view([150 12]);
axis([-0.6 0.6 -0.6 0.6 0 1.35]);
camva(9);
daspect([1 1 1]);
load lbr_waypoints.mat
cdt = 0.001; 
tt = 0:cdt:5;

[qDesired, qdotDesired, qddotDesired, tt] = exampleHelperJointTrajectoryGeneration(tWaypoints, qWaypoints, tt);

n = size(qDesired,1);
tauFeedForward = zeros(n,7);
for i = 1:n
    tauFeedForward(i,:) = inverseDynamics(lbr, qDesired(i,:), qdotDesired(i,:), qddotDesired(i,:));
end

weights = [0.3,0.8,0.6, 0.6,0.3,0.2,0.1];
Kp = 100*weights;
Kd = 2* weights;

once = 1;

feedForwardTorque = zeros(n, 7);
pdTorque = zeros(n, 7);
timePoints = zeros(n,1);
Q = zeros(n,7);
QDesired = zeros(n,7);

for i = 1:n
    % Get joint state from Vrep.
    jsMsg = receive(jointStateSub);
    data = jsMsg.Data;
    
    % Parse the received message. 
    % The Data in jsMsg is a 1-by-15 vector.
    % 1:7  - joint positions
    % 8:14 - joint velocities
    % 15   - time (Gazebo sim time) when the joint state is updated
    q = double(data(1:7))';
    qdot = double(data(8:14))';
    t = double(data(15));
    
    % Set the start time.
    if once
        tStart = t;
        once = 0;
    end
    
    % Find the corresponding index h in tauFeedForward vector for joint 
    % state time stamp t.
    h = ceil((t - tStart + 1e-8)/cdt);
    if h>n
        break
    end
    
    % Log joint positions data.
    Q(i,:) = q';
    QDesired(i,:) = qDesired(h,:);
    
    % Inquire feed-forward torque at the time when the joint state is
    % updated (Gazebo sim time).
    tau1 = tauFeedForward(h,:);
    % Log feed-forward torque.
    feedForwardTorque(i,:) = tau1;
    
    % Compute PD compensation torque based on joint position and velocity
    % errors.
    tau2 = Kp.*(qDesired(h,:) - q) + Kd.*(qdotDesired(h,:) - qdot);
    % Log PD torque.
    pdTorque(i,:) = tau2';
    
    % Combine the two torques.
    tau = tau1 + tau2;
    
    % Log the time.
    timePoints(i) = t-tStart;
    
    % Send torque to Gazebo.
    %jtMsg.Data = tau;
    %send(jointTauPub,jtMsg);    
end