function controlleur = generateContolleur(drone)

F_barre = null(drone.Fb);
% M_barre = null(drone.Mb);
% 
% rank(drone.Fb)
% rank(drone.Mb)
%
% rank(drone.Mb*F_barre);
% rank(drone.Fb);
% 
% v1 = M_barre(:,1);
% v2 = M_barre(:,2);

K = F_barre*F_barre';
% drone.Mb*K*drone.Mb'
% inv(drone.Mb*K*drone.Mb')


kappa = norm(drone.Fb*[1;1;0;0]);
u_barre = [1;1;0;0]/kappa; %on observe que u_barre = -0.7139 * v1 donc u_barre appartient au ker(M)
d_etoile = drone.Fb*u_barre;
skew_sym_d_etoile = skew_sym(d_etoile);



% M_k = pinv(drone.Mb);
M_k = K*drone.Mb'*(drone.Mb*K*drone.Mb');
% drone.Fb*M_k
% norm(drone.Fb*u_barre);
% drone.Mb*M_barre
%drone.Mb*u_barre

%% initialize controlleur structure
controlleur = struct('kap',1.0, ...
                     'kad',1.0, ...
                     'kpp',1.0, ...
                     'kpd',1.0, ...
                     'k_delta',2.0, ...
                     'u_barre',u_barre, ...
                     'M_k',M_k, ...
                     'skew_sym_d_etoile',skew_sym_d_etoile, ...
                     'd_etoile', d_etoile ...
                     );
%drone.Fb*controlleur.M_k
end