function [theta1, theta2] = scara_ik_xy(p,a1,a2)

c2 = (p(1)^2 + p(2)^2 -a1^2 - a2^2)/(2*a1*a2);
s2 = sqrt(1-c2^2);
theta2 = atan2(s2,c2);

alpha=atan2(p(2),p(1)); 
beta = acos((p(1)^2+p(2)^2+a1^2-a2^2)/(2*a1*sqrt(p(1)^2+p(2)^2)));
theta1 = alpha-beta;

end


