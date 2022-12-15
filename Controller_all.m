function [ U , err,err_rpy]= Controller_all(Rel_X,Ref_Rel_X,~)
%[ U , err] = Controller_all([Rel_trans ; Rel_q ; Rel_wCT]) 
%U   = input 6 X 1 Vector
%err = store err vector


% Required Natural Frequency   = wn 
% Required Damping Coefficient = z
% Above requirements are achieved from Required Settling Time and Maximum Overshoot
 
%% Translation
    err_trans = Rel_X(1:6) - Ref_Rel_X(1:6) ;
    wn1 =[2 2 2]*.1;
    z1  = [.7 .7 .9];
    Tr_Kp = wn1.^2;
    Tr_Kd = 2*wn1.*z1;
   
    U_Tr = -Tr_Kp'.*err_trans(1:3)-Tr_Kd'.*err_trans(4:6);

%% Rotation    
    err_q = qmult(Rel_X(7:10),qinv(Ref_Rel_X(7:10)));
    err_q = err_q/norm(err_q);
    err_q_matlab = [err_q(4,1);err_q(1:3,1)];
    [err_yaw , err_pitch ,err_roll] = quat2angle(err_q_matlab');
    
    err_w =Rel_X(11:13)-Ref_Rel_X(11:13);
    wn2 = [5 5 5]*.0010;
    z2  = [.7 .7 .7];
    Ro_Kp=wn2.^2;
    Ro_Kd=2*wn2.*z2;
    U_Ro = -Ro_Kp'.*[err_roll ; err_pitch ; err_yaw ]-Ro_Kd'.*err_w;

%%     
    U= [ U_Tr ; U_Ro ] ;    
    err=[err_trans; err_q; err_w];
    err_rpy=[err_roll ; err_pitch ; err_yaw ];

%% 
    %{
        %Inputs as in ECI 
         %~~~~~Given U_temp1 is in Target Frame~~~~~
         %U must be converted to ECI 
         U1  = conv_to_ECI(U(1:3,1),XT(1:6,1));
         U2  = conv_to_ECI(U(4:6,1),XT(1:6,1));
        %Required Forces wrt ECI
         Req_Forces = U1*Sat_Mass/1000; %because U1 is acc in Km/sec2
         
        U = [ U1 ; U2 ] ;
   %}
end