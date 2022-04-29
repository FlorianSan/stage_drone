function drone = generatequad()
drone = struct('MASS',               0.5,      ... % ok: 0.438
               'INERTIA',            eye(3),     ... % below
               'G',                  9.81,       ... % ok
               'Mb',                 zeros(3,4), ...
               'Fb',                 zeros(3,4) ...
               ); % ok
%% acording to inertia identificayiton (python + IMU measurements)
drone.INERTIA = diag([0.075 0.075 0.15]);

%% matrice d'effort et moment
cf = 10e-5;
ct = 10e-7;
l = 0.5;

drone.Fb = [0 0 0 0; 0 0 0 0; cf cf cf cf];

% Calcul du moment
drone.Mb = [0 -l*cf 0 l*cf; -l*cf 0 l*cf 0; ct -ct ct -ct];

end