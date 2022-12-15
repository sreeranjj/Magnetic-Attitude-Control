function qmult = qmult(q1,q2)
%strictly considering q(1:3) as Vector part and q(4) as scalar 

q1_cross = [ q1(4)*eye(3)-x_cross(q1(1:3)) ,  q1(1:3);
                -q1(1:3)'                 ,    q1(4)];

qmult =q1_cross*q2;
end