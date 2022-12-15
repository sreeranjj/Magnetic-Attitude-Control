function Q=dcmtoQ(R)
%this function will convert Rotation Matrix R to Quarternion Q
%
%Quarternion will be of form:
%Column Vector Q = [  q1;
%                     q2;
%                     q3;
%                     q4] where q4 is scaler

tr=trace(R);

if( tr > 0 )
    
    q4 = sqrt(tr+1)/2;
    q1 =( R(3,2) - R(2,3) )/(4*q4);
    q2 =( R(1,3) - R(3,1) )/(4*q4);
    q3 =( R(2,1) - R(1,2) )/(4*q4);

elseif (R(1,1)>R(2,2) && R(1,1)>R(3,3))   
    
    q1 = sqrt(1+R(1,1)-tr)/2;
    q2 =( R(1,2) + R(2,1) )/(4*q1);
    q3 =( R(1,3) + R(3,1) )/(4*q1);
    q4 =( R(3,2) - R(2,3) )/(4*q1);

elseif R(2,2) > R(3,3)
    
    q2 = sqrt(1+R(2,2)-tr)/2;
    q1 =( R(1,2) + R(2,1))/(4*q2);
    q3 =( R(2,3) + R(3,2))/(4*q2);
    q4 =( R(1,3) - R(3,1))/(4*q2);

else
    q3 = sqrt(1+R(3,3)-tr)/2;
    q1 =( R(1,3) + R(3,1) )/(4*q3); 
    q2 =( R(2,3) + R(3,2) )/(4*q3);
    q4 =( R(2,1) - R(1,2) )/(4*q3);
end

Q=[q1;q2;q3;q4];

if q4 < 0
   Q=-Q;
%{
elseif q4 == 0
    if q1 < 0
        Q=-Q;
    elseif q1 == 0
        if q2 < 0
           Q=-Q;
        elseif q2 == 0
            if q3 < 0
               Q=-Q; 
            else
                
            end
        end
    end
%}    
end
Q=Q/norm(Q);

end
