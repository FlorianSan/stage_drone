function x_sat = sat(x, M)
    a = M;
    b = -log(1/2);
    c = 1;
    x_sat = 2*a*exp(-b*exp(-c*x)) -a;
end