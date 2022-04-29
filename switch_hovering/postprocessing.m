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
width=500;
height=1800;
set(gcf,'position',[10,10,width,height])
%sgtitle("Position du drone dans l'espace")
[ha, pos] = tight_subplot(6,1,.03,.05,.1);
axes(ha(1)); 
plot(out.tout, out.x(:,1),'b', 'linewidth', 2)
hold on
plot(out.tout, out.x(:,2), 'r', 'linewidth', 2)
plot(out.tout, out.x(:,3), 'g', 'linewidth', 2)
% plot([0,50,50.01,51], [50,50,0,0], 'b:')
% plot([0,50,50.01,51], [25,25,0,0], 'r:')
% plot([0,50,50.01,51], [12.5,12.5,0,0], 'g:')
hold off
ylim([-30 60])
grid on
ylabel('[m]')
legend('p_{x}','p_{y}','p_{z}','p_{rx}','p_{ry}','p_{rz}','Orientation','horizontal','Location','south')
set(gca,'Xticklabel',[])
%xlabel('time [s]')

axes(ha(2));
plot(out.tx, out.orientation_deg(:,1), 'linewidth', 2)
hold on
plot(out.tx, out.orientation_deg(:,2), 'linewidth', 2)
plot(out.tx, out.orientation_deg(:,3), 'linewidth', 2)
grid on
ylabel('[deg]')
ylim([-20 140])
legend('\phi','\theta','\psi','Orientation','horizontal','Location','northwest')
set(gca,'Xticklabel',[])

axes(ha(3));
plot(out.tx, out.u(:,1), 'linewidth', 2)
hold on
plot(out.tx, out.u(:,2), 'linewidth', 2)
grid on
ylabel('[rpm]')
legend('\omega_{1}','\omega_{2}','Orientation','horizontal','Location','northwest')
set(gca,'Xticklabel',[])

axes(ha(4));
plot(out.tx, out.u(:,3), 'linewidth', 2)
hold on
plot(out.tx, out.u(:,4), 'linewidth', 2)
grid on
ylabel('[deg]')
ylim([-35 35])
legend('\delta_{1}','\delta_{2}','Orientation','horizontal','Location','northwest')
set(gca,'Xticklabel',[])

axes(ha(5));
plot(out.tx, out.Vx, 'linewidth', 2)
grid on
% ylabel('[ ]')
ylim([-500 6000])
legend('V_{x}','Orientation','horizontal')
set(gca,'Xticklabel',[])



axes(ha(6));
plot(out.tx, out.r, 'linewidth', 2)
grid on
% ylabel('[ ]')
ylim([-1 2])
leg1 = legend('$\ell$ - discret state','Orientation','horizontal');
set(leg1,'Interpreter','latex');
%set(gca,'Xticklabel',[])


% figure(3)
% plot(out.tx, out.jx)
% yline(0,':','Évolution non linéaire');
% yline(1,':','Évolution linéaire');
% yline(2,':','Évolution non linéaire','LabelHorizontalAlignment','left');
% yline(3,':','Évolution linéaire','LabelHorizontalAlignment','left');
% ylim([-1 4])
% grid on
% ylabel('Sauts')
% xlabel('time [s]')

