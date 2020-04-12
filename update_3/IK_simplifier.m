syms x y z
syms theta1 theta2 theta3

Y4 = 0.11651167932822327;
Z4 = 0;
Y2 = 0.05149697649731044;
Y3 = 0.07238155732083357;

le = sqrt(Y4*Y4+Z4*Z4);

theta1 = atan(x/abs(y));
L1 = sqrt(x*x+y*y);
L2 = (L1-Y2)*(L1-Y2);
L = sqrt(z*z+L2);
alpha1 = abs(atan((L1-Y2)/z));
alpha2 = acos((L*L+Y3*Y3-le*le)/(2*L*Y3));
theta2 = alpha1+alpha2-pi()/2;
beta = acos((le*le+Y3*Y3-L*L)/(2*le*Y3));
gamma = atan(Z4/Y4);
theta3 = beta-gamma;
% return [theta1,-theta2,theta3*2]
vpa(simplify(theta1))
vpa(simplify(theta2))
vpa(simplify(theta3))