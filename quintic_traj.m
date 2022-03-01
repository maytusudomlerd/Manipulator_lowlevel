syms t qi qf dqi dqf ddqi ddqf c0 c1 c2 c3 c4 c5 real

coef = [c0;c1;c2;c3;c4;c5];
param = [qi;qf;dqi;dqf;ddqi;ddqf];
k = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 2 0 0 0;
    1 t t^2 t^3 t^4 t^5;
    0 1 2*t 3*t^2 4*t^3 5*t^4;
    0 0 2 6*t 12*t^2 20*t^3]

and = inv(k) * param
