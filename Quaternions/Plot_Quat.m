%Show all Quarternions
q_all=zeros(i_tot,4);
w_all=zeros(i_tot,3);
q_norm_all=zeros(i_tot,1);

for i=1:i_tot
  [R,w]=get_Rot_w_ECI(XT_trans_all(i,:)');
  q_all(i,:) = dcmtoQ(R)';
  w_all(i,:)=w';
  %q_all(i,:)= dcm2quat(R);
    q_norm_all(i)=norm(q_all(i,:));
end
%Quartenions at Each Step vs time 
hold off;
figure;
plot(q_norm_all);
figure(1);
    subplot(4,1,1);  
    plot(q_all(:,1),'*');
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
    plot(q_all(:,2),'*');
    hold off;
    
    subplot(4,1,3);  
    hold on;
    grid on;
    xlabel('Time in Seconds');
    ylabel('q(3)');
    plot(q_all(:,3),'*');
    hold off;
    
    subplot(4,1,4);  
    hold on;
    grid on;
    xlabel('Time in Seconds');
    ylabel('q(4)');
    plot(q_all(:,4),'*');
    hold off;
figure(3);
    subplot(3,1,1);
    hold on;
    plot(w_all(:,1));
    grid on;
    xlabel('Time in Seconds');
    ylabel('w_x');
    hold off;
    
    subplot(3,1,2);
    hold on;
    plot(w_all(:,2));
    grid on;
    xlabel('Time in Seconds');
    ylabel('w_y');
    hold off;
    
    subplot(3,1,3);
    hold on;
    plot(w_all(:,3));
    grid on;
    xlabel('Time in Seconds');
    ylabel('w_z');
    hold off;