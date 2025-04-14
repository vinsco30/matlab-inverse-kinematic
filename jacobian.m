function J = jacobian(th1, th2, a1, a2)

    J = [-a2*sin(th1+th2)- a1*sin(th1), -a2*sin(th1 + th2), 0, 0;
        a2*cos(th1 + th2) + a1*cos(th1),  a2*cos(th1 + th2), 0, 0;
        0,                  0, 1, 0;
        1,                  1, 0, 1];
end