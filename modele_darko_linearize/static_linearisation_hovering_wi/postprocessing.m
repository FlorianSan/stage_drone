%--------------------------------------------------------------------------
% Matlab M-file Project: HyEQ Toolbox @  Hybrid Systems Laboratory (HSL), 
% https://hybrid.soe.ucsc.edu/software
% http://hybridsimulator.wordpress.com/
% Filename: postprocessing_ex1_2a.m
%--------------------------------------------------------------------------
% Project: 
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   See also HYEQSOLVER, PLOTARC, PLOTARC3, PLOTFLOWS, PLOTHARC,
%   PLOTHARCCOLOR, PLOTHARCCOLOR3D, PLOTHYBRIDARC, PLOTJUMPS.
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.3 Date: 05/20/2015 3:42:00

close all 


% Position
figure(1)
x0=10;
y0=10;
width=500;
height=1800;
set(gcf,'position',[x0,y0,width,height])
%sgtitle("Position du drone dans l'espace")
[ha, pos] = tight_subplot(5,1,.03,.05,.1);
axes(ha(1)); 
plot(out.tout, out.positions(:,1),'b')
hold on
plot(out.tout, out.positions(:,2), 'r')
plot(out.tout, out.positions(:,3), 'g')

% plot(out.tout, out.x1(:,1),'b--')
% plot(out.tout, out.x1(:,2), 'r--')
% plot(out.tout, out.x1(:,3), 'g--')
% 
plot([0,50], [3,3], 'b:')
plot([0,50], [4,4], 'r:')
plot([0,50], [5,5], 'g:')
hold off
ylim([-2 9])
grid on
ylabel('[m]')
legend('p_{cx}','p_{cy}','p_{cz}', 'p_{sx}','p_{sy}','p_{sz}','Orientation','horizontal','Location','north')
set(gca,'Xticklabel',[])
%xlabel('time [s]')

axes(ha(2));
plot(out.tout, out.orientation_deg(:,1),'b')
hold on
plot(out.tout, out.orientation_deg(:,2),'r')
plot(out.tout, out.orientation_deg(:,3),'g')
grid on
ylabel('[deg]')
%ylim([-180 180])
legend('\phi_{c}','\theta_{c}','\psi_{c}','Orientation','horizontal','Location','south')
set(gca,'Xticklabel',[])

axes(ha(3));
plot(out.tout, out.u(:,1))
hold on
plot(out.tout, out.u(:,2))
grid on
ylabel('[rpm]')
legend('\omega_{1}','\omega_{2}','Orientation','horizontal')
set(gca,'Xticklabel',[])

axes(ha(4));
plot(out.tout, out.u(:,3))
hold on
plot(out.tout, out.u(:,4))
grid on
ylabel('[deg]')
ylim([-35 35])
legend('\delta_{1}','\delta_{2}','Orientation','horizontal')
set(gca,'Xticklabel',[])

axes(ha(5));
plot(out.tout, out.wind(:,1))
hold on
plot(out.tout, out.wind(:,2))
plot(out.tout, out.wind(:,3))
grid on
ylabel('[m.s^{-1}]')
ylim([-2 9])
legend('w_{x}','w_{y}','w_{z}','Orientation','horizontal')
% set(gca,'Xticklabel',[])

% figure(1)
% x0=10;
% y0=10;
% width=500;
% height=500;
% set(gcf,'position',[x0,y0,width,height])
% %sgtitle("Position du drone dans l'espace")
% [ha, pos] = tight_subplot(3,1,.04,.05,.1);
% axes(ha(1)); 
% plot(out.tout, out.positions(:,1),'b')
% hold on
% plot(out.tout, out.positions(:,2), 'r')
% plot(out.tout, out.positions(:,3), 'g')
% plot([0,12], [8,8], 'b:')
% plot([0,12], [9,9], 'r:')
% plot([0,12], [10,10], 'g:')
% hold off
% %ylim([-30 30])
% grid on
% ylabel('[m]')
% legend('p_{x}','p_{y}','p_{z}','p_{rx}','p_{ry}','p_{rz}','Orientation','horizontal','Location','north')
% set(gca,'Xticklabel',[])
% %xlabel('time [s]')
% 
% axes(ha(2));
% plot(out.tout, unwrap(out.orientation_deg(:,1)))
% hold on
% plot(out.tout, unwrap(out.orientation_deg(:,2)))
% plot(out.tout, unwrap(out.orientation_deg(:,3)))
% grid on
% ylabel('[deg]')
% %ylim([-180 180])
% legend('\phi','\theta','\psi','Orientation','horizontal')
% set(gca,'Xticklabel',[])
% 
% axes(ha(3));
% plot(out.tout, out.Vx(:,1))
% grid on
% ylabel('[ ]')
% %ylim([-180 180])
% legend('V(x)','Orientation','horizontal')
% % set(gca,'Xticklabel',[])
