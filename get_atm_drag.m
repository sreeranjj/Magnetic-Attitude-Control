function atm_drag = get_atm_drag(X)
 global Re Sat_Mass
   R_vec=X(1:3,1)*1000; %in meters 
   V_vec=X(4:6,1)*1000; %in m/sec
   q = X(7:10,1);
   Rot_Mat = Qtodcm(q);
 we = [0 ; 0;.000072921158553]; %rad/sec
 n  = [  1, 0, 0;
         0, 1, 0;
         0, 0, 1;
        -1, 0, 0;
         0,-1, 0;
         0, 0,-1]; 
     
 S = ones(6,1); % in m^2 

Vrel_ECI=V_vec+cross(we,R_vec);
Vrel=Rot_Mat*Vrel_ECI;
Cd=1.28;
h=norm(R_vec/1000)-Re;
rho = get_density(h);
atm_drag = 0; 

for i=1:6
    proj = n(i,:)*Vrel;
    if(proj>0)
        Drag = -rho*Cd*Vrel*S(i)*proj/2; 
    else
        Drag=[0;0;0];
    end
    atm_drag=atm_drag+Drag;
end

atm_drag= (Rot_Mat')*atm_drag/Sat_Mass/1000; % km/sec2
end