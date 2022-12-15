function R = Qtodcm(Q)
%this function will give Rotation Matrix from Quarternions
%Quarternion must be of form:
%Column Vector Q = [  q1;
%                     q2;
%                     q3;
%                     q4] where q4 is scaler
%if QuatCheck(Q)
    
    norm(Q);
    Q=Q/norm(Q);
    q  = Q(1:3,1);
    q4 = Q(4);
    Q_cross=[    0,  -q(3),   q(2);
              q(3),      0,  -q(1);
             -q(2),   q(1),     0];
    I = eye(3);   
    R = ((q4^2)-(q')*q)*I + 2*(q*q') - 2*q4*Q_cross;
%end

end