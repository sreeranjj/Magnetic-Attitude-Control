function x_cross= x_cross(x)
%{
Given 3 X 1 vector x
This function gives its As name suggests ...;)
%}
x_cross = [  0 , -x(3) ,  x(2);
           x(3),     0 , -x(1);
          -x(2),  x(1) ,     0];
      
end