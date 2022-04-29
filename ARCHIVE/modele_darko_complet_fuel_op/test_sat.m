
x = linspace(-5,5);
plot(x, sat(x, 1.44))
hold on
plot(x, atan(x))
plot([-2,2], [-2,2])
hold off

syms ep ev kp kd a b c

f = -(2*a*exp(-b*exp(-c*kp*(ep+ev*sqrt(abs(ev)/(2*a)+kd/kp)))) - a);
% f = exp(-exp(-x));
f_dot = diff(f)

x=1
a = 1;
b = -log(1/2);
c = 2;
subs(f_dot)

ep = [1;0;0];
ev = [0;0;0];
QTO = sat(controlleur.kpp * (ep + ev .* sqrt((abs(ev)/(2*controlleur.a)).^2 + (controlleur.kpd/controlleur.kpp)^2)), controlleur.a);
f_r = drone.MASS * [0;0;drone.G] - QTO;