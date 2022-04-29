function v = quadprod(q1, q2)

    v = [q1(1)*q2(1) - q1(2:4)'*q2(2:4); q1(1)*q2(2:4) + q2(1)*q1(2:4) + cross(q1(2:4), q2(2:4))];