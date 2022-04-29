
% yaw = deg2rad(0);
% pitch = deg2rad(-90);
% roll = deg2rad(8);
% q1 = ToQuaternion(yaw, pitch, roll);

a = 0.3; %-0.7071 à 0.7071
q = [sqrt(1/2-a^2); a; (2*a^2-1)/(2*sqrt(1/2-a^2)); a];
norm(q)
eulZYZ = rad2deg(quat2eul(q',"ZYZ"))


Rq = q2dcm(q);

v = [1;0;0];
rot_mat = Rq*v

q0 = q(1);
q_v = q(2:4);
t = 2 * cross(q_v, v);

rot_quat = v + q0*t + cross(q_v, t)

diff = rot_mat-rot_quat


B = 0.0;
g=9.81;

officiel = [0 2*sqrt(2)*g 0;
-sqrt(2)*g 0 sqrt(2)*g ;
0 -sqrt(2)*g 0 ];

eta = sqrt(1/2 - B^2);
e1 = B;
e2 = (2*B^2-1)/(2*sqrt(1/2-B^2));
e3 = B;

F_test = g*[ 0 -4*e2 -4*e3;
    2*e2-4*e1*e3/2*sqrt(1-e1^2 - e2^2 - e3^2) 2*e1-4*e2*e3/2*sqrt(1-e1^2 - e2^2 - e3^2)  -4*e3*e3/2*sqrt(1-e1^2 - e2^2 - e3^2) + 2*sqrt(1-e1^2 - e2^2 - e3^2);
    2*e3+4*e1*e2/2*sqrt(1-e1^2 - e2^2 - e3^2) -(4*e2*e2)/(2*sqrt(1-e1^2 - e2^2 - e3^2)) - 2*sqrt(1 - e1^2 - e2^2 - e3^2) 2*e1 + 4*e3*e2/2*sqrt(1-e1^2 - e2^2 - e3^2)];

disp(officiel)
disp(F_test)
