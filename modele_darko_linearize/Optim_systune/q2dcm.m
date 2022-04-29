function dcm = q2dcm(q)

eta = q(1);
eps = q(2:4);

dcm = eye(3) + 2*eta*skew_sym(eps) + 2*(eps*eps' - eps'*eps*eye(3));

