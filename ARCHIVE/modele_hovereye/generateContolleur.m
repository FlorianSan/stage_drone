function controlleur = generateContolleur()


%% initialize controlleur structure
controlleur = struct('k1',0.5, ...
                     'k2',2.1, ...
                     'kf',1.0, ...
                     'kn',2.4, ...
                     'k_delta',7, ...
                     'km',4.5, ...
                     'kr',8.5 ...
                     );
%drone.Fb*controlleur.M_k
end