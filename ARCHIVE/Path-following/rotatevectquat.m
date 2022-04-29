function v = rotatevectquat(q, r)
    %Abbreviations for the various angular functions
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    
    A = [(1-2*q2^2-2*q3^2), 2*(q1*q2 +q0*q3), 2*(q1*q3-q0*q2);
         2*(q1*q2-q0*q3), (1-2*q1^2 - 2*q3^2), 2*(q2*q3+q0*q1);
         2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), (1-2*q1^2-2*q2^2)
        ];
    
    v = A*r;