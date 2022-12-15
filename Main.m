%% Translation Motion MAIN

%% Initializations
close all
clc
clear 

%Add necessary folder for quaternion conversion and Arithmetic
addpath('Quaternions');
%Atmoshere Density Model


global GM Re T Sat_Mass M

%Time Step in seconds
T = 1;

M = load('EXP_Atm_Model');

%GM is constant for earth and relativly small sized satellites
GM = 3.986004418*(10^5);    %in Km^3/sec^2

%Radius Of EARTH
Re = 6371;                  %in kms


%Total_iterations 
 i_tot = 6001;
%Time Stamp Array ::
 t_stmp = 0:T:i_tot; 

%Satellite Mass of Chaser (obviously  
 Sat_Mass= 1000; %in Kgs
%Inertia Matrix 
    J    = [1000 ,   0 ,    0;
               0 , 900 ,    0;
               0 ,   0 , 1200 ];  %in Kg-m2
    invJ = J^(-1); %to reduce calculation time of inverse after Each Iteration
    %to send above both as parameters each time
    J_all = [ J ; invJ];

Sat_dim=.001;%in Kilometers    

%Max force in single direction of Body fixed axis
Force_max = 5; %in Newtons

%4 thruster are placed in along each direction ::
    %2 along positive
    %2 along Negative
thrust_max= 5/2; %in Newtons

%Considering Each thruster to be equally displaced from CG
%Pependicular distance between 2 thrusters
thrus_dis=Sat_dim;

%Max acceleration From Thrusters Possible 
Max_acc = Force_max/(Sat_Mass*1000); %in km/sec2

%Max Torque in single direction of Body fixed axis
Torque_max = thrus_dis*thrust_max*2*[1;1;1] ; %  d*F  %And 2 Because other Two thrusters will also help

%Max_Ang Rotational Acceleration
Max_alpha= invJ*Torque_max;

%States
  %Translation State X :: [Rx , Ry , Rz, Vx , Vy ,Vz] in ECI
  %Rotation State X :: [q , w ] in ECI  
  %q = quaternion wrt ECI 
  %w = body rotation rates wrt Body


    %initialize  Target State (Column Vector) 
      %Translation:
        trans_XT = [6800000;0;0;0;7659.26;0]/1000;
        
      %Rotation
        %initial Quaternion target ( Rotation ) Same As hill's frame Will
        %Probably remain Same
        [RotT , wini]= get_Rot_w_ECI(trans_XT); %First we Find the Rotation matrix from ECI to hill frame
        q_ini = dcmtoQ(RotT);                 %Second we Convert Rotation matrix to Quaternions
        rot_XT = [q_ini ; wini];        
        
     XT=[trans_XT;rot_XT];

    %initialize Translation Chaser State (Column Vector) 
        %Translation
        trans_XC = [6799925,203.73901147096,0,0,7659.175585,0]'/1000;
       
        %Rotation 
        %for Chaser ,q is considered relative to hill's frame 
         %initial Quaternion ( Rotation )
          q_rel = [0.6853 ; 0.6953 ; 0.1531 ;0.1531];
         %initial angular Rates wrt Hill'frame  
          w_rel = [0.0; 0.0 ;0.0011];
          rot_XC=[q_rel;w_rel];
          rot_X_relini=rot_XC;
        
    XC = [trans_XC;rot_XC];
          

%Desired Relative states
        k=2;
         %Translation
            trans_Ref_Rel_X =[0 ; 0 ; 0 ; 0 ; 0 ; 0];
         %Rotation
            %For now : desired RELATIVE quaternion is the identity quaternion
            q_des_rel = [ 0 ; 0 ; 0 ; 1];
            %angular rates 
            w_des_Rel = [ 0 ; 0 ; 0];
            %net Desired Rotation state 
            rot_Ref_Rel_X  = [q_des_rel ; w_des_Rel];

    Ref_Rel_X = [trans_Ref_Rel_X ; rot_Ref_Rel_X];
    
%PwPfm Params 
    %Translation
        n    = 10;    %No of thrust toggle per second
        dt   = 1/n;   %in seconds
        km   = 10;    %forward loop gain in low Pass filter
        tm   = .1;    %in seconds
        Uon  =  8; 
        Uoff = .5;
        Um   =  1; 
        f_tra = [0,0,0];
        f_rot = [0,0,0];
    %Rotation
        
  
%Matrices to Collect DATA
    
    XT_all        = zeros(i_tot,13);    %Store From Actual Plant for Target
    XC_all        = zeros(i_tot,13);    %Store From Actual Plant for Chaser 
    Rel_X_all     = zeros(i_tot,13);    
    RatesC_all    = zeros(i_tot,13);
    Force_all     = zeros(i_tot,3);
    U_all         = zeros(i_tot,6);
    Ref_Rel_X_all = zeros(i_tot,13);
    err_all       = zeros(i_tot,13);
    err_rpy_all   = zeros(i_tot,3);
    U_T           = zeros(6,1);
    Torque_all    = zeros(i_tot,3);
    U_PWPFM_all   = zeros(i_tot*n,6);
    
%% Simulation Loop

for i = 1 : i_tot 
    %%  Sensor Outputs
        
        %Camera :: Relative Attitude (q has been propagated)
            Rel_q = qmult(XC(7:10),qinv(XT(7:10))); %To map Vectors in Target Frame to Vectors in Chaser frame
            Rel_q = Rel_q/norm(Rel_q);
            A = Qtodcm(Rel_q);                      %To map Vectors in Target Frame to Vectors in Chaser frame
        
        %Gyros :: Inertial w for Chaser And Target in Body Frame
            wT_I = XT(11:13,1);
            wC_I = XC(11:13,1);
            Rel_wCT = wC_I - A*wT_I; 
            
        %Camera :: Relative Position And Velocity            
            Rel_trans_C_T = get_PVA_C_to_T(XT(1:6,1),XC(1:6,1));  
             Rel_pos = Rel_trans_C_T(1:3,1); %these are Actual Inputs wrt Chaser
             Rel_vel = Rel_trans_C_T(4:6,1) ;
        
        Rel_X=[ Rel_pos;Rel_vel; Rel_q; Rel_wCT ];
        
    %% Store Data - 1
       %States
        XT_all(i,:) = XT';
        XC_all(i,:) = XC';
       %Rel States
        Rel_X_all(i,:) = Rel_X';    
       
       %Reference Relative
        Ref_Rel_X_all(i,:)= Ref_Rel_X;
        
    %% Controller And  PWPFM

        [U_temp1 , err,err_rpy] = Controller_all(Rel_X,Ref_Rel_X,XT);
     
       %PWPFM
         U_tr = U_temp1(1:3,1)/Max_acc; 
         U_rot= U_temp1(4:6,1)./Max_alpha;
       
        [Ux_tra , f_tra(1)] = PWPFM(U_tr(1),dt, km, tm, Uon, Uoff, Um ,n,f_tra(1));
        [Uy_tra , f_tra(2)] = PWPFM(U_tr(2),dt, km, tm, Uon, Uoff, Um ,n,f_tra(2));
        [Uz_tra , f_tra(3)] = PWPFM(U_tr(3),dt, km, tm, Uon, Uoff, Um ,n,f_tra(3));
        [Ux_rot , f_rot(1)] = PWPFM(U_rot(1),dt, km, tm, Uon, Uoff, Um ,n,f_rot(1));
        [Uy_rot , f_rot(2)] = PWPFM(U_rot(2),dt, km, tm, Uon, Uoff, Um ,n,f_rot(2));
        [Uz_rot , f_rot(3)] = PWPFM(U_rot(3),dt, km, tm, Uon, Uoff, Um ,n,f_rot(3));
        
        U_PWPFM=[Ux_tra ,Uy_tra ,Uz_tra, Ux_rot, Uy_rot, Uz_rot]; % in 1's 0's and -1's
        
        U_C_Tr(1,1)  = mean((U_PWPFM(:,1)*Max_acc));
        U_C_Tr(2,1)  = mean((U_PWPFM(:,2)*Max_acc));
        U_C_Tr(3,1)  = mean((U_PWPFM(:,3)*Max_acc));
        U_C_Rot(1,1) = (mean(U_PWPFM(:,4)))*Max_alpha(1);
        U_C_Rot(2,1) = (mean(U_PWPFM(:,5)))*Max_alpha(2);
        U_C_Rot(3,1) = (mean(U_PWPFM(:,6)))*Max_alpha(3);
        U_PWPFM_all(i*n-n+1:i*n,:)=U_PWPFM;
        app_Tor = J*Max_alpha;
        app_F =U_C_Tr*Sat_Mass;
        
    %% Actual plant - Both Target And Chaser
       %Inputs as in ECI
         %~~~~~Given U_temp1 is in Target Frame~~~~~
         %U must be converted to ECI 
         RotMat_T = Qtodcm(XT(7:10,1));
         U_C_Tr=conv_to_ECI(U_C_Tr,XT(1:6,1));
         U1  = U_C_Tr;
         U2  = U_C_Rot;
         
         U_C=[ U1; U2];
         
        [XT_next,RatesT]  = Rk4_all(XT,U_T,T,J_all);
        [XC_next,RatesC]  = Rk4_all(XC,U_C,T,J_all);
    
    %% Store Data - 2     
       
       %Controller Outputs 
           %Force
            Force_all(i,:)=app_F';
           %Final input Chaser
            U_all(i,:) = U_C';
           %Error
           err_all(i,:) = err';
           err_rpy_all(i,:)=err_rpy';
       %Plant Output
        %Accelerations of Satellites In ECI
            
         RatesC_all(i,:) = RatesC';
            
     %% Update State (XT,XC - only used By Actual Plant )
      
        XT = XT_next;
        XC = XC_next;  
    
end


%% PLOTING Results
 %% Translation

%Some Animations (If You Want to make Video of Trajectories Incase...):
        %{
        h=animatedline('Color','b');
        g=animatedline('Color','r');
        hold on;
        grid on;
        title('Path Analysis');
        xlabel('X (ECI) in Kms');
        ylabel('Y (ECI) in Kms');
        zlabel('Z (ECI) in Kms');
        pbaspect([1,1,1]);

        for i =1:i_tot
        addpoints(h,XT_all(i,1),XT_all(i,2),XT_all(i,3));
        addpoints(g,XC_all(i,1),XC_all(i,2),XC_all(i,3));
        drawnow
        end
        %}

%Path Plots : 
        %{
        figure(1);
        hold on;
        grid on;
        title('Path Analysis');
        xlabel('X (ECI) in Kms');
        ylabel('Y (ECI) in Kms');
        zlabel('Z (ECI) in Kms');
        pbaspect([1,1,1]);
        plot3(XT_all(:,1),XT_all(:,2),XT_all(:,3));
        plot3(XC_all(:,1),XC_all(:,2),XC_all(:,3)); 
        hold off;
        %}
% Radius of Target around earth
        
        for i = 1:i_tot
            Radius(i) = norm(XT_all(i,1:3));
        end
        


%Actual Acceleration Target
    %{ 
    figure(8);
        subplot(3,1,1);  %for Actual Acc X
        hold on;
        title('Actual Acceleration Target')
        grid on;
        xlabel('Time in Seconds');
        ylabel('X in Km/sec^2');
        plot(Actual_Rates_all(:,4));
        hold off;

        subplot(3,1,2);  %for Actual Acc Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Y in Km/sec^2');
        plot(Actual_Rates_all(:,5));
        hold off;

        subplot(3,1,3);  %for Actual Acc Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Z in Km/sec^2');
        plot(Actual_Rates_all(:,6));
        hold off;
    %}
%From Actual Plant Relative trajectories of Chaser wrt Target
    %
        figure(2);
        hold on;
        grid on;
        title('Relative Path Exact');
        xlabel('X (T-frame) in Kms');
        ylabel('Y (T-frame) in Kms');
        zlabel('Z (T-frame) in Kms');
        pbaspect([1,1,1]);
        plot3(Rel_X_all(:,1),Rel_X_all(:,2),Rel_X_all(:,3));
        hold off;
     %}
    
%Relative States - Actual Plant vs Desired Relative
      %Translation
      figure(5);

        subplot(3,2,1);  % Pos X
        plot(Rel_X_all(:,1));
        grid on;
        hold on;
        plot(Ref_Rel_X_all(:,1),'--')
        title('Relative States')
        xlabel('Time in Seconds');
        ylabel('X in Kms');
        hold off;

        subplot(3,2,2);  % Velocity X
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Vel_x in Km/sec');
        plot(Rel_X_all(:,4));
        plot(Ref_Rel_X_all(:,4),'--');
        hold off;

        subplot(3,2,3);  %for Diff in Ref Position Pos Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Y in Kms');
        plot(Rel_X_all(:,2));
        plot(Ref_Rel_X_all(:,2),'--');
        hold off;

        subplot(3,2,4);  %for Diff in Ref Velocity Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Vel_y in Km/sec');
        plot(Rel_X_all(:,5));
        plot(Ref_Rel_X_all(:,5),'--');
        hold off;

        subplot(3,2,5);  %for Diff in Ref Position Pos Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Z in Kms');
        plot(Rel_X_all(:,3));
        plot(Ref_Rel_X_all(:,3),'--');
        hold off;

        subplot(3,2,6);  %for Diff in Ref Velocity Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Vel_x in Km/sec');
        plot(Rel_X_all(:,6));
        plot(Ref_Rel_X_all(:,6),'--');
        hold off;
    %}
%Error in State From Reference Output      
      %
        figure(6);

        subplot(3,2,1);  %for Diff in Ref Position Pos X
        plot(err_all(:,1));
        grid on;
        hold on;
        title('Error')
        xlabel('Time in Seconds');
        ylabel('X in Kms');
        hold off;

        subplot(3,2,2);  %for Diff in Ref Velocity X
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Vel_x in Km/sec');
        plot(err_all(:,4));
        hold off;

        subplot(3,2,3);  %for Diff in Ref Position Pos Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Y in Kms');
        plot(err_all(:,2));
        hold off;


        subplot(3,2,4);  %for Diff in Ref Velocity Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Vel_y in Km/sec');
        plot(err_all(:,5));
        hold off;

        subplot(3,2,5);  %for Diff in Ref Position Pos Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Z in Kms');
        plot(err_all(:,3));
        hold off;

        subplot(3,2,6);  %for Diff in Ref Velocity Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Vel_x in Km/sec');
        plot(err_all(:,6));
        hold off;
    %}
%Plot Forces
    %
        figure(7);
        subplot(3,1,1);  %for Thrust X
        hold on;
        title('Thrusts')
        grid on;
        xlabel('Time in Seconds');
        ylabel('X in Km/sec^2');
        plot(Force_all(:,1));
        hold off;

        subplot(3,1,2);  %for Thrust Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Y in Km/sec^2');
        plot(Force_all(:,2));
        hold off;

        subplot(3,1,3);  %for Thrust Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Z in Km/sec^2');
        plot(Force_all(:,3));
        hold off;
    %}
%Plot for Input from Controller  to Plant ANd changes in between 
    %
        figure(8);
        subplot(3,1,1);  %for Input_U X
        hold on;
        title('Input from Controller')
        grid on;
        xlabel('Time in Seconds');
        ylabel('X in Km/sec^2');
        plot(U_all(:,1));
        hold off;

        subplot(3,1,2);  %for Input_U Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Y in Km/sec^2');
        plot(U_all(:,2));
        hold off;

        subplot(3,1,3);  %for Input_U Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Z in Km/sec^2');
        plot(U_all(:,3));
        hold off;

        
    %Plots For  PWPFM Output
    %
        figure(9);

        subplot(3,1,1);  %for  X
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('X input');
        plot(U_PWPFM_all(:,1));
        title('PWPFM Output');
        hold off;

        subplot(3,1,2);  %for  Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Y input');
        plot(U_PWPFM_all(:,2));
        hold off;

        subplot(3,1,3);  %for  Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Z Input');
        plot(U_PWPFM_all(:,3));
        hold off;
    %}
    
 %% Rotation Plots 
 
 
    %Relative Quaternions at Each Step vs time 
    %
        figure(10);
        subplot(4,1,1);  
        plot(Rel_X_all(:,7));
        grid on;
        hold on;
        title('Quaternions')
        xlabel('Time in Seconds');
        ylabel('q(1)');
        hold off;

        subplot(4,1,2); 
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('q(2)');
        plot(Rel_X_all(:,8));
        hold off;

        subplot(4,1,3);  
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('q(3)');
        plot(Rel_X_all(:,9));
        hold off;

        subplot(4,1,4);  
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('q(4)');
        plot(Rel_X_all(:,10));
        hold off;
    %}
    %Angular Velocity Relative
    figure(11);
        subplot(3,1,1);  
        plot(Rel_X_all(:,11));
        grid on;
        hold on;
        title('Angular Velocity')
        xlabel('Time in Seconds');
        ylabel('wx');
        hold off;

        subplot(3,1,2); 
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('wy');
        plot(Rel_X_all(:,12));
        hold off;

        subplot(3,1,3);  
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('wz');
        plot(Rel_X_all(:,13));
        hold off;
        
%Plot for Input from Controller  to Plant 
    %
        figure(12);
        subplot(3,1,1);  %for Input_U X
        hold on;
        title('Control_angular_acc Input from Controller')
        grid on;
        xlabel('Time in Seconds');
        ylabel('X in rad/sec^2');
        plot(U_all(:,4));
        hold off;

        subplot(3,1,2);  %for Input_U Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Y in rad/sec^2');
        plot(U_all(:,5));
        hold off;

        subplot(3,1,3);  %for Input_U Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Z in rad/sec^2');
        plot(U_all(:,6));
        hold off;

        
    %Plots For  PWPFM Output
    %
        figure(13);

        subplot(3,1,1);  %for  X
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('X input');
        plot(U_PWPFM_all(:,4));
        title('PWPFM Output');
        hold off;

        subplot(3,1,2);  %for  Y
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Y input');
        plot(U_PWPFM_all(:,5));
        hold off;

        subplot(3,1,3);  %for  Z
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('Z Input');
        plot(U_PWPFM_all(:,6));
        hold off;
    %}
    figure(14);
        subplot(3,1,1);  
        plot(err_rpy_all(:,1));
        grid on;
        hold on;
        title('Error in roll pitch yaw in Radians')
        xlabel('Time in Seconds');
        ylabel('Roll');
        hold off;

        subplot(3,1,2); 
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('pitch');
        plot(err_rpy_all(:,2));
        hold off;

        subplot(3,1,3);  
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('yaw');
        plot(err_rpy_all(:,3));
        hold off;
   
  % Target VS Chaser
   
   %Quaternion comparison
    figure(15);
        subplot(4,1,1);  
        plot(XT_all(:,7),'--');
        grid on;
        hold on;
        title('Quaternions');
        xlabel('Time in Seconds');
        ylabel('q(1)');
        plot(XC_all(:,7));
        hold off;

        subplot(4,1,2); 
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('q(2)');
        plot(XT_all(:,8),'--');
        plot(XC_all(:,8));
        hold off;

        subplot(4,1,3);  
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('q(3)');
        plot(XT_all(:,9),'--');
        plot(XC_all(:,9));
        hold off;

        subplot(4,1,4);  
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('q(4)');
        plot(XT_all(:,10),'--');
        plot(XC_all(:,10));
        hold off;
    %}
    %Angular Velocity 
    figure(16);
        subplot(3,1,1);  
        plot(XT_all(:,11),'--');
        grid on;
        hold on;
        title('Angular Velocity')
        xlabel('Time in Seconds');
        ylabel('wx');
        plot(XC_all(:,11));
        hold off;

        subplot(3,1,2); 
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('wy');
        plot(XT_all(:,12),'--');
        plot(XC_all(:,12));
        hold off;

        subplot(3,1,3);  
        hold on;
        grid on;
        xlabel('Time in Seconds');
        ylabel('wz');
        plot(XT_all(:,13),'--');
        plot(XC_all(:,13));
        hold off;