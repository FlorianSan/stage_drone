function dxdt  = drone_dyn(x, wind, u, drone)

%% physical constants
% gravity
g = drone.G;
% vehicle mass
m = drone.MASS;
% vehicle inertia matrix
J = drone.INERTIA; % [3x3 real symmetric] in body axis

%% state demultiplexing
% p = x(1:3);
v = x(4:6);
q  = x(7:10);
wb = x(11:13);

Rq = q2dcm(q);

[Fp, Mp] = external_force_moment(x, u, wind, drone);

% position
dpdt = v;

% kinematics
dvdt =  -[0;0;g] + (1/m)*Rq*drone.Fb*u + (1/m)*Rq*Fp; 

%% quaternion derivative
eta = q(1);
eps = q(2:4);

dqdt = (1/2)*[-eps'; eta*eye(3) + skew_sym(eps)]*wb;

%% angular velocity derivative
% kinematics
dwdt = -J\cross(wb, J*wb) + J\(drone.Mb*u) + J\Mp;


%% derivative multiplexing
dxdt = [dpdt; dvdt; dqdt; dwdt] ;