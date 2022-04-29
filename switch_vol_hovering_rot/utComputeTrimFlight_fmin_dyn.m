function trimVal = utComputeTrimFlight_fmin_dyn(V_trim, drone)


    function F = setTrim(u)
        g = drone.G;
        % vehicle mass
        m = drone.MASS;
        % vehicle inertia matrix
        J = drone.INERTIA;
        q = [sqrt(1-u(3)^2 - u(4)^2 - u(5)^2 );  u(3); u(4); u(5)];
        x = [0;0;0; V_trim;0;0; q; 0;0;0];
        cmd = [u(1); u(1); u(2); u(2)];
        
        Rq = q2dcm(q);
        
        [Fp, Mp] = external_force_moment(x, cmd, 0, drone);
        dvdt = -[0;0;g] + (1/m)*Rq*drone.Fb*cmd + (1/m)*Rq*Fp; 
        dwdt = J\(drone.Mb*cmd) + J\Mp;
        
        F(1) = dvdt(1);
        F(2) = dvdt(2);
        F(3) = dvdt(3);
        F(4) = dwdt(1);
        F(5) = dwdt(2);
        F(6) = dwdt(3);
	end

    function F = setTrimFzero(u)
        F = norm(setTrim(u));
    end
    T = drone.MASS*drone.G;
    delta = -0.1;
%     eps2 = -sqrt(2);
    eps2 = -0.01;
    eps1 = -0.01;
    eps3 = -0.01;
    
    if 1 %false
        options = optimoptions('fsolve','Algorithm','levenberg-marquardt');
        [trimVal,fval,exitFlag] = fsolve(@setTrim,[T; delta; eps1; eps2; eps3],options);
    else
        trimVal = fminsearch(@setTrimFzero, [T; delta; eps1; eps2; eps3]);
    end    
    
end