clear all ;
close all;
clc;

drone = generateDarko();
controlleur = generateContolleur(drone);

speed = 20;

p_initial = [0; 0; 0];
v_initial = [10; 0; 0];
omega_initial = [0; 0; 0];


yaw = deg2rad(0);
pitch = deg2rad(0);
roll = deg2rad(0);
q_initial = ToQuaternion(yaw, pitch, roll);

initial_state = [p_initial; v_initial; q_initial; omega_initial];

p1 = [0;0;0];
p2 = [1000; 1000; 1000];

affichage = 0;

if affichage == 1
    figure(1)
    %sgtitle("Position du drone dans l'espace")
    subplot(3,1,1)
    plot(out.tout, out.positions(:,1))
    ylim([-100 1000])
    grid on
    ylabel('px [m]')
    xlabel('time [s]')

    subplot(3,1,2)
    plot(out.tout, out.positions(:,2))
    ylim([-100 1000])
    grid on
    ylabel('py [m]')
    xlabel('time [s]')

    subplot(3,1,3)
    plot(out.tout, out.positions(:,3))
    ylim([-100 1000])
    grid on
    ylabel('pz [m]')
    xlabel('time [s]')
    
        figure(2)
    %sgtitle("Position du drone dans l'espace")
    subplot(3,1,1)
    plot(out.tout, out.orientation_deg(:,1))
    %ylim([-1 0.6])
    grid on
    ylabel('Roulis [°]')
    xlabel('time [s]')

    subplot(3,1,2)
    plot(out.tout, out.orientation_deg(:,2))
    %ylim([-130 -40])
    grid on
    ylabel('Tangage [°]')
    xlabel('time [s]')

    subplot(3,1,3)
    plot(out.tout, out.orientation_deg(:,3))
    %ylim([-1 6])
    grid on
    ylabel('Lacet [°]')
    xlabel('time [s]')
    
            figure(3)
    %sgtitle("Position du drone dans l'espace")
    subplot(4,1,1)
    plot(out.tout, out.commande(:,1))
    %ylim([600 850])
    grid on
    ylabel('\omega droite [rad/s]')
    xlabel('time [s]')

    subplot(4,1,2)
    plot(out.tout, out.commande(:,2))
    %ylim([-850 -600])
    grid on
    ylabel('\omega gauche [rad/s]')
    xlabel('time [s]')

    subplot(4,1,3)
    plot(out.tout, out.commande(:,3))
    %ylim([-20 35])
    grid on
    ylabel('\delta droit [deg]')
    xlabel('time [s]')
    
    subplot(4,1,4)
    plot(out.tout, out.commande(:,4))
    %ylim([-20 35])
    grid on
    ylabel('\delta gauche [deg]')
    xlabel('time [s]')
    
    figure(4)
    plot3(out.positions(:,1),out.positions(:,2), out.positions(:,3));
    hold on
    plot3([p1(1),p2(1)], [p1(2),p2(2)], [p1(3),p2(3)]);
    grid on;
    xlabel('X [m]')
    ylabel('Z [m]')
    legend("Position du drone", "Ligne position initiale-cible", 'Location','northwest')
    
    data=[out.tout, out.positions, out.orientation_visu];
    h=Aero.Animation;
    f=figure;
    h.Figure=f;
    h.initialize();
    h.FramesPerSecond=10;
    h.TimeScaling = 5;
    h.createBody('astredwedge.mat')
    h.bodies{1}.TimeseriesSourceType='Array6DoF';
    h.bodies{1}.timeseriesSource=data;
    h.show()
    h.VideoRecord = 'on';
    h.VideoQuality = 50;
    h.VideoCompression = 'Motion JPEG AVI';
    h.VideoFilename = 'Vol';
    h.play()
    h.VideoRecord='off';
end