function qinv = qinv(q)
%this function function quaternion inverse 
qinv=qconj(q)/(qnorm(q)^2);
end