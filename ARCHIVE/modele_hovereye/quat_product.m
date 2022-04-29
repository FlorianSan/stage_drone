function q3 = quat_product(q1, q2)
%#codegen

eta1 = q1(1);
eps1 = q1(2:4);

A_q1 = [eta1 -eps1'; eps1 eta1*eye(3)+skew_sym(eps1)];
q3 = A_q1*q2;