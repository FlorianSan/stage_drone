function matrix = skew_sym(u)
% input u = [1;1;1]
matrix = [  0   -u(3)  u(2); 
           u(3)   0   -u(1);
          -u(2)  u(1)   0   ];
