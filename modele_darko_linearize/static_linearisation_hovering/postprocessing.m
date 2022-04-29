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

%darko_control_lineaire1

% figure(1)
% x0=10;
% y0=10;
% width=500;
% height=500;
% set(gcf,'position',[x0,y0,width,height])
% %sgtitle("Position du drone dans l'espace")
% [ha, pos] = tight_subplot(3,1,.04,.05,.1);
% axes(ha(1)); 
% plot(out.tout, out.x(:,1),'b', 'linewidth', 2)
% hold on
% plot(out.tout, out.x(:,2), 'r', 'linewidth', 2)
% plot(out.tout, out.x(:,3), 'g', 'linewidth', 2)
% 
% plot(out.tout, out.x1(:,1),'b--', 'linewidth', 2)
% plot(out.tout, out.x1(:,2), 'r--', 'linewidth', 2)
% plot(out.tout, out.x1(:,3), 'g--', 'linewidth', 2)
% 
% plot([0,12], [4,4], 'b:', 'linewidth', 2)
% plot([0,12], [5,5], 'r:', 'linewidth', 2)
% plot([0,12], [6,6], 'g:', 'linewidth', 2)
% hold off
% ylim([-5 10])
% grid on
% ylabel('[m]')
% legend('p_{cx}','p_{cy}','p_{cz}', 'p_{sx}','p_{sy}','p_{sz}','Orientation','horizontal','Location','north')
% set(gca,'Xticklabel',[])
% %xlabel('time [s]')
% 
% axes(ha(2));
% plot(out.tout, out.orientation_deg(:,1),'b', 'linewidth', 2)
% hold on
% plot(out.tout, out.orientation_deg(:,2),'r', 'linewidth', 2)
% plot(out.tout, out.orientation_deg(:,3),'g', 'linewidth', 2)
% 
% plot(out.tout, out.orientation_deg(:,1),'b--', 'linewidth', 2)
% plot(out.tout, out.orientation_deg(:,2),'r--', 'linewidth', 2)
% plot(out.tout, out.orientation_deg(:,3),'g--', 'linewidth', 2)
% grid on
% ylabel('[deg]')
% %ylim([-180 180])
% legend('\phi_{c}','\theta_{c}','\psi_{c}','\phi_{s}','\theta_{s}','\psi_{s}','Orientation','horizontal','Location','south')
% set(gca,'Xticklabel',[])
% 
% axes(ha(3));
% plot(out.tout, out.Vx(:,1), 'linewidth', 2)
% grid on
% % ylabel('[ ]')
% ylim([-50 150])
% legend('V(x)','Orientation','horizontal')
% % set(gca,'Xticklabel',[])

figure(1)
x0=10;
y0=10;
width=600;
height=400;
set(gcf,'position',[x0,y0,width,height])
%sgtitle("Position du drone dans l'espace")
[ha, pos] = tight_subplot(2,1,.04,.05,.1);
axes(ha(1)); 
plot(out.tout, out.x(:,1),'b', 'linewidth', 2)
hold on
plot(out.tout, out.x(:,2), 'r', 'linewidth', 2)
plot(out.tout, out.x(:,3), 'g', 'linewidth', 2)
plot([0,12], [8,8], 'b:', 'linewidth', 2)
plot([0,12], [9,9], 'r:', 'linewidth', 2)
plot([0,12], [10,10], 'g:', 'linewidth', 2)
hold off
%ylim([-30 30])
grid on
ylabel('[m]')
legend('p_{x}','p_{y}','p_{z}','p_{rx}','p_{ry}','p_{rz}','Orientation','horizontal','Location','north')
set(gca,'Xticklabel',[])
%xlabel('time [s]')

axes(ha(2));
plot(out.tout, unwrap(out.orientation_deg(:,1)), 'linewidth', 2)
hold on
plot(out.tout, unwrap(out.orientation_deg(:,2)), 'linewidth', 2)
plot(out.tout, unwrap(out.orientation_deg(:,3)), 'linewidth', 2)
grid on
ylabel('[deg]')
%ylim([-180 180])
legend('\phi','\theta','\psi','Orientation','horizontal')
% set(gca,'Xticklabel',[])
% 
% axes(ha(3));
% plot(out.tout, out.Vx(:,1), 'linewidth', 2)
% grid on
% % ylabel('[ ]')
% %ylim([-180 180])
% legend('V(x)','Orientation','horizontal')
% % set(gca,'Xticklabel',[])
