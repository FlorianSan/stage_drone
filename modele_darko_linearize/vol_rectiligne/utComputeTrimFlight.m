function trimVal = utComputeTrimFlight(V_inf_x, drone)
    Swet   = drone.WET_SURFACE;
    Sdry   = drone.DRY_SURFACE;
    Sp = pi*drone.PROP_RADIUS^2;
    Thetam = drone.ELEVON_MEFFICIENCY(2);
    Thetaf = drone.ELEVON_FEFFICIENCY(2);
    rho = drone.RHO;
    S = Swet + Sdry;
    m = drone.MASS;
    g = drone.G;
    
    PHI_fv = drone.PHI(1:3,1:3);
    Cd_0 = PHI_fv(1,1);
    die_c = PHI_fv(3,3);
    
    eta = sqrt( norm(V_inf_x)^2); %+ phi_n*norm(B*wb)^2 
    
    function F = find_eps2(x)
        F(1) = sqrt(1-x^2)*x*(rho*S*eta*die_c*V_inf_x*(Thetaf/Thetam-1)) + (1-2*x)*(m*g-1/4*rho*S*eta*Thetaf*die_c*V_inf_x);
    end
    function F = find_eps2zero(u)
            F = norm(find_eps2(u));
    end
    eps2 = fminsearch(@find_eps2zero, -0.4);

    eps2_ = (1/(rho*S*eta*die_c*V_inf_x*(Thetaf/Thetam-1)-2*m*g + 1/2*rho*S*eta*Thetaf*die_c*V_inf_x))*(-m*g + 1/4*rho*S*eta*Thetaf*die_c*V_inf_x);
    
    discriminent = (2*sqrt(1-eps2^2)*eps2*m*g - 1/2*rho*S*eta*Thetaf*Cd_0*(1-2*eps2^2)*V_inf_x)^2 - 8*(1-Swet/4*Sp)*(-(rho*S*eta)^2/Swet * Thetaf/Thetam*(1-2*eps2^2)*sqrt(1-eps2^2)*eps2*V_inf_x^2);
    T1 = (-(2*sqrt(1-eps2^2)*eps2*m*g - 1/2*rho*S*eta*Thetaf*Cd_0*(1-2*eps2^2)*V_inf_x) + sqrt(discriminent))/(4*(1-Swet/4*Sp));
    T2 = (-(2*sqrt(1-eps2^2)*eps2*m*g + 1/2*rho*S*eta*Thetaf*Cd_0*(1-2*eps2^2)*V_inf_x) + sqrt(discriminent))/(4*(1-Swet/4*Sp));
    
    T = max([T1;T2]);
    delta = (-2*Sp*rho*S*eta*sqrt(1-eps2^2)*eps2)/(T*Swet*Thetam);
   trimVal = [T; delta; eps2; eps2_];
    
end