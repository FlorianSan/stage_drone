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

figure(2)
plot(out.tx,out.jx)

figure(3)
plot(out.r(:,1),out.r(:,2), 'DisplayName','ref')
hold on
plot(out.x(:,1),out.x(:,2), 'DisplayName','traj')
xlabel("p1")
ylabel("p2")
legend()
axis equal



figure(4)
subplot(4,1,1)
plot(out.tx, out.x(:,5))
grid on
ylabel('s[s]')
xlabel('time [s]')

subplot(4,1,2)
plot(out.tx, out.x(:,6))
grid on
ylabel('spoint [-]')
xlabel('time [s]')

subplot(4,1,3)
plot(out.tx, out.x(:,7))
grid on
ylabel('q [s]')
xlabel('time [s]')

subplot(4,1,4)
plot(out.tx, out.Vi(:,1) + out.Vi(:,2))
grid on
ylabel('V_{loc} [-]')
xlabel('time [s]')


figure(5)

subplot(4,1,1)
plot(out.tx, out.u(:,1))
grid on
ylabel('u1 [s]')
xlabel('time [s]')

subplot(4,1,2)
plot(out.tx, out.u(:,2))
grid on
ylabel('u2 [s]')
xlabel('time [s]')

subplot(4,1,3)
plot(out.tx, out.u(:,3))
grid on
ylabel('w [s]')
xlabel('time [s]')

subplot(4,1,4)
plot(out.tx, out.sigma)
grid on
ylabel('sigma [s]')
xlabel('time [s]')


figure(6)

subplot(4,1,1)
plot(out.tx, out.e(:,1))
grid on
ylabel('ep1 [s]')
xlabel('time [s]')

subplot(4,1,2)
plot(out.tx, out.e(:,2))
grid on
ylabel('ep2 [s]')
xlabel('time [s]')

subplot(4,1,3)
plot(out.tx, out.e(:,3))
grid on
ylabel('ev1 [s]')
xlabel('time [s]')

subplot(4,1,4)
plot(out.tx, out.e(:,4))
grid on
ylabel('ev2 [s]')
xlabel('time [s]')

figure(7)
subplot(4,1,1)
plot(out.tx, out.x(:,1))
grid on
ylabel('p1 [s]')
xlabel('time [s]')

subplot(4,1,2)
plot(out.tx, out.x(:,2))
grid on
ylabel('p2 [s]')
xlabel('time [s]')

subplot(4,1,3)
plot(out.tx, out.x(:,3))
grid on
ylabel('v1 [s]')
xlabel('time [s]')

subplot(4,1,4)
plot(out.tx, out.x(:,4))
grid on
ylabel('v2 [s]')
xlabel('time [s]')

