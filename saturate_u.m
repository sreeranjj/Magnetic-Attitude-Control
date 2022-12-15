function input_u_dir = saturate_u(input_u_dir,Max_acc)
%This Function saturate the input from Controller
%Since input from Controller may have required accelerations which are beyond Physical boundaries
% input_u_dir = input in X y or z dir 
% Max_acc = max possible Acceleration

if input_u_dir >= Max_acc
        input_u_dir=Max_acc;

else if input_u_dir <= (-Max_acc)
        input_u_dir= -Max_acc;
     end
end