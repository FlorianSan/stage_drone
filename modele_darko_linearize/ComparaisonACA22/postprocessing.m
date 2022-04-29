close all 
% 
% 
% % Position
% figure(1)
% %sgtitle("Position du drone dans l'espace")
% subplot(3,1,1)
% plot(out.tout, out.sv_spectral(:,1), 'b')
% % ylim([-10 120])
% hold on 
% plot(out.tout, out.sv_systune(:,1),'r')
% grid on
% legend('Spectral abscissa','Systune')
% ylabel('px [m]')
% xlabel('time [s]')
% 
% subplot(3,1,2)
% plot(out.tout, out.sv_spectral(:,2), 'b')
% % ylim([-10 120])
% hold on 
% plot(out.tout, out.sv_systune(:,2),'r')
% grid on
% legend('Spectral abscissa','Systune')
% ylabel('py [m]')
% xlabel('time [s]')
% 
% subplot(3,1,3)
% plot(out.tout, out.sv_spectral(:,3), 'b')
% % ylim([-10 120])
% hold on 
% plot(out.tout, out.sv_systune(:,3),'r')
% grid on
% legend('Spectral abscissa','Systune')
% ylabel('pz [m]')
% xlabel('time [s]')
% 
% figure(2)
% plot(out.tout, out.sv_spectral(:,9), 'b')
% hold on
% plot(out.tout, out.sv_systune(:,9), 'r')
% % yline(250,'-.r');
% % yline(400,'-.r','Switch');
% % ylim([-500 4500])
% grid on
% legend('Spectral abscissa','Systune')
% ylabel('\epsilon_{2}')
% xlabel('time [s]')
% 
% figure(3)
% tiledlayout(4,1)
% 
% bx1 = nexttile;
% plot(out.tout,out.cmd_spectral(:,1), 'b')
% hold on
% plot(out.tout,out.cmd_systune(:,1), 'r')
% grid on
% legend('Spectral abscissa','Systune')
% title("T1")
% ylabel('Trust [N]')
% xlabel('time [s]')
% 
% bx2 = nexttile;
% plot(out.tout,out.cmd_spectral(:,2), 'b')
% hold on
% plot(out.tout,out.cmd_systune(:,2), 'r')
% grid on
% legend('Spectral abscissa','Systune')
% title("T2")
% ylabel('Trust [N]')
% xlabel('time [s]')
% 
% bx3 = nexttile; 
% plot(out.tout,out.cmd_spectral(:,3), 'b')
% hold on
% plot(out.tout,out.cmd_systune(:,3), 'r')
% grid on
% legend('Spectral abscissa','Systune')
% title("d1")
% ylabel('Elevon deflection [°]')
% xlabel('time [s]')
% 
% bx4 = nexttile;
% plot(out.tout,out.cmd_spectral(:,4), 'b')
% hold on
% plot(out.tout,out.cmd_systune(:,4), 'r')
% grid on
% legend('Spectral abscissa','Systune')
% title("d2")
% ylabel('Elevon deflection [°]')
% xlabel('time [s]')
% 
% figure(4)
% plot(out.tout, out.wind(:,1), 'b')
% % yline(250,'-.r');
% % yline(400,'-.r','Switch');
% % ylim([-500 4500])
% grid on
% legend('Horizontal wind')
% ylabel('\omega_{x} [m.s^{-1}]')
% xlabel('time [s]')
% 

figure(5)
tiledlayout(6,1)

ax1 = nexttile;
% ylim([-10 120])
plot(out.tout, out.sv_spectral(:,1), 'b', 'LineWidth', 2)
hold on 
plot(out.tout, out.sv_systune(:,1),'r', 'LineWidth', 2)
y_limit = ylim;
fill([30 30 50 50],[y_limit(1), y_limit(2)+2, y_limit(2)+2, y_limit(1)],0.7*ones(1,3),'FaceAlpha',0.3, 'EdgeColor','none')
grid on
title("Horizontal position")
% legend('Spectral abscissa','Systune', 'Location','northoutside')
ylabel('px [m]')
% xlabel('time [s]')

ax2 = nexttile;

plot(out.tout, out.sv_spectral(:,3), 'b', 'LineWidth', 2)
% ylim([-10 120])
hold on 
plot(out.tout, out.sv_systune(:,3),'r', 'LineWidth', 2)
ylim([-0.1 1.2])
y_limit = ylim;
fill([30 30 50 50],[y_limit(1), y_limit(2), y_limit(2), y_limit(1)],0.7*ones(1,3),'FaceAlpha',0.3, 'EdgeColor','none')
grid on
title("Vertical position")
% legend('Spectral abscissa','Systune')
ylabel('pz [m]')
% xlabel('time [s]')

ax3 = nexttile;
plot(out.tout, out.spectral_orientation_deg(:,2), 'b', 'LineWidth', 2)
hold on
plot(out.tout, out.systune_orientation_deg(:,2), 'r', 'LineWidth', 2)
y_limit = ylim;
fill([30 30 50 50],[y_limit(1), y_limit(2), y_limit(2), y_limit(1)],0.7*ones(1,3),'FaceAlpha',0.3, 'EdgeColor','none')
grid on
title("Tilt angle")
% legend('Spectral abscissa','Systune')
ylabel('\theta [°]')
% xlabel('time [s]')

ax4 = nexttile;
plot(out.tout,2*out.cmd_spectral(:,2), 'b', 'LineWidth', 2)
hold on
plot(out.tout,2*out.cmd_systune(:,2), 'r', 'LineWidth', 2)
y_limit = ylim;
fill([30 30 50 50],[y_limit(1)-2, y_limit(2), y_limit(2), y_limit(1)-2],0.7*ones(1,3),'FaceAlpha',0.3, 'EdgeColor','none')
grid on
% legend('Spectral abscissa','Systune')
title("Trust")
ylabel('T [N]')
% xlabel('time [s]')

ax5 = nexttile;

plot(out.tout,out.cmd_spectral(:,3), 'b', 'LineWidth', 2)
hold on
plot(out.tout,out.cmd_systune(:,3), 'r', 'LineWidth', 2)
ylim([-35 35])
y_limit = ylim;
fill([30 30 50 50],[y_limit(1), y_limit(2), y_limit(2), y_limit(1)],0.7*ones(1,3),'FaceAlpha',0.3, 'EdgeColor','none')
grid on
% legend('Spectral abscissa','Systune')
title("Elevon deflection")
ylabel(' \delta [°]')
% xlabel('time [s]')

ax6 = nexttile;
plot(out.tout, -out.wind(:,1), 'color', 'k', 'LineWidth', 2)
hold on
ylim([-1 5])
y_limit = ylim;
fill([30 30 50 50],[y_limit(1), y_limit(2), y_limit(2), y_limit(1)], 0.7*ones(1,3),'FaceAlpha',0.3, 'EdgeColor','none')
grid on
title("Horizontal wind")
ylabel('\omega_{x} [m.s^{-1}]')
xlabel('time [s]')
