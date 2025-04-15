clear 
close all

syms th1 th2 d3 th4 a1 a2 d1 d0 real

AB0 = [eye(3), [0 0 d0]'; 0 0 0 1];

A01 = [cos(th1) -sin(th1) 0 a1*cos(th1);
    sin(th1) cos(th1) 0 a1*sin(th1);
    0 0 1 0;
    0 0 0 1];

A12 = [cos(th2) -sin(th2) 0 a2*cos(th2);
    sin(th2) cos(th2) 0 a2*sin(th2);
    0 0 1 0;
    0 0 0 1];

A23 = [eye(3), [0 0 d3]';
    0 0 0 1];

A34 = [cos(th4) -sin(th4) 0 0;
    sin(th4) cos(th4) 0 0;
    0 0 1 0;
    0 0 0 1];

A4e = [0 1 0 0;
    1 0 0 0;
    0 0 -1 0;
    0 0 0 1];

T = simplify(AB0*A01*A12*A23*A34*A4e);
T_step1 = A01*A12;
T_step2 = T_step1*A23;

z0 = AB0(1:3,3);
p0 = AB0(1:3,4);
pe = simplify(T(1:3,4));

z1 = simplify(A01(1:3,3));
z2 = simplify(T_step1(1:3,3));
z3 = simplify(T_step2(1:3,3));

p1 = simplify(A01(1:3,4));
p2 = simplify(T_step1(1:3,4));
p3 = simplify(T_step2(1:3,4));

J = [cross(z0,pe-p0), cross(z1,pe-p1), z2, cross(z3,pe-p3);
    z0, z1, zeros(3,1), z3];

J = simplify(J);

J_rid = [J(1:3,:); J(6,:)];


