function [RotMat , w]= get_Rot_w_ECI(X)
%This Function Gives Rotation MAtrix and Rotation Rate of Hills frame of Satellite in X state 
%------------------This Rotation Matrix is Not of Body Fix Frame--------------------
%because we need to find Reference frame wrt target , we use its orbit to find [i j k]  

    R_vec=X(1:3)';  %Row Matrix
    V_vec=X(4:6)';  %Row Matrix

    R=norm(R_vec);  

    %h=Specific Angular momentum of Satellite in its Orbit --- Row Matrix
    h_vec=cross(R_vec,V_vec);
    h=norm(h_vec);

    i=R_vec/R;
    k=h_vec/h;
    j=cross(k,i);

    RotMat = [i;j;k];
    w = (h_vec/(R^2))'; %Col Vector
end