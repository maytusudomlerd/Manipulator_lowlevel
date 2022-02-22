syms dt x1 x2 p11 p12 p21 p22 theta w Q R real

%from model
A = [1 dt;0 1];
C = [1 0];
G = [0.5 * dt^2 ; dt ];

%state output
Xk = [x1;x2];
%covariance output
Pk = [p11 p12;p21 p22];
%input state
y = [1 0]*[theta ; w];
%predict
Xp = A * Xk;
Pp = (A * Pk * A') + (G * Q * G');
Yp = C * Xp;
%update
Y_update = y - Yp;
Sk =  (C * Pp * C') + R;
Kk = Pp * C' / Sk;
%output
X_out = simplify(Xp + (Kk * Y_update));
P_out = simplify((eye(2) - Kk * C) * Pp);

x1 = simplify(X_out(1))
x2 = simplify(X_out(2))
p11 = simplify(P_out(1,1))
p12 = simplify(P_out(1,2))
p21 = simplify(P_out(2,1))
p22 = simplify(P_out(2,2))

