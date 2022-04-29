function controlleur = generateContolleur(drone)

%  F_barre = null(drone.Fb);
% M_barre = null(drone.Mb);
% 
% rank(F_barre)
% rank(drone.Fb);
% rank(drone.Mb);
% 
% rank(drone.Mb*F_barre);
% rank(drone.Fb);


kappa = norm(drone.Fb*[1;1;0;0]);
u_barre = [1;1;0;0]/kappa; %on observe que u_barre = -0.7139 * v1 donc u_barre appartient au ker(M)
d_etoile = drone.Fb*u_barre;
skew_sym_d_etoile = skew_sym(d_etoile);



M_k = pinv(drone.Mb);
% M_k(1,1) = 0;
% M_k(2,1) = 0;
% 
% M_k(3,3) = 0;
% M_k(4,3) = 0;
% drone.Fb*M_k;
% norm(drone.Fb*u_barre);
% drone.Mb*M_k;
% drone.Mb*u_barre;

%% initialize controlleur structure
controlleur = struct('kap',2.0, ... %gain proportionnel erreur d'orientation
                     'kad',2.0, ... %gain dérivatif erreur d'orientation
                     'kpp',1.0, ...  %gain proportionnel erreur de position
                     'kpd',1.0, ... %gain dérivatif erreur de vitesse
                     'k_delta',1.0, ... %gain de convergence exponentielle f_delta
                     'u_barre',u_barre, ...
                     'M_k',M_k, ...
                     'skew_sym_d_etoile',skew_sym_d_etoile, ...
                     'd_etoile', d_etoile ...
                     );
%drone.Fb*controlleur.M_k
end