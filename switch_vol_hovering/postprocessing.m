%--------------------------------------------------------------------------
% Matlab M-file Project: HyEQ Toolbox @  Hybrid Systems Laboratory (HSL), 
% https://hybrid.soe.ucsc.edu/software
% http://hybridsimulator.wordpress.com/
% Filename: postprocessing_ex1_2a.m
%--------------------------------------------------------------------------
% Project: Simulation of a hybrid system (bouncing ball)
% Description: postprocessing for the bouncing ball example
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   See also HYEQSOLVER, PLOTARC, PLOTARC3, PLOTFLOWS, PLOTHARC,
%   PLOTHARCCOLOR, PLOTHARCCOLOR3D, PLOTHYBRIDARC, PLOTJUMPS.
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.3 Date: 05/20/2015 3:42:00

close all 

% plot solutions
% figure(1)
% for k = 1:length(out.tx)
%     plot(out.r(k,1),out.r(k,2), 'DisplayName','ref')
%     hold on
%     plot(out.x(k,1),out.x(k,2), 'DisplayName','traj')
%     xlabel("p1")
%     ylabel("p2")
%     legend()
%     axis equal
% end

% Position
figure(1)
%sgtitle("Position du drone dans l'espace")
subplot(3,1,1)
plot(out.tx, out.x(:,1))
ylim([-10 120])
grid on
ylabel('px [m]')
xlabel('time [s]')

subplot(3,1,2)
plot(out.tx, out.x(:,2))
ylim([-1 1])
grid on
ylabel('py [m]')
xlabel('time [s]')

subplot(3,1,3)
plot(out.tx, out.x(:,3))
ylim([-10 5])
grid on
ylabel('pz [m]')
xlabel('time [s]')

figure(2)
plot(out.tx, out.Vx)
yline(250,'-.r');
yline(400,'-.r','Switch');
ylim([-500 4500])
xlim([0 80])
grid on
ylabel('V(x)')
xlabel('time [s]')

figure(3)
plot(out.tx, out.jx)
yline(0,':','Évolution non linéaire');
yline(1,':','Évolution linéaire');
yline(2,':','Évolution non linéaire','LabelHorizontalAlignment','left');
yline(3,':','Évolution linéaire','LabelHorizontalAlignment','left');
ylim([-1 4])
grid on
ylabel('Sauts')
xlabel('time [s]')
