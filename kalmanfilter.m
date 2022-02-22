function [X1,X2,P11,P12,P21,P22] = kalmanfilter(R,Q,dt,theta,x1,x2,p11,p12,p21,p22)
X1 = (4*R*x1 + 4*p11*theta + 4*dt^2*p22*theta + 4*R*dt*x2 + 4*dt*p12*theta + 4*dt*p21*theta + Q*dt^4*theta)/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22);
X2 = x2 - (((Q*dt^3)/2 + p22*dt + p21)*(x1 - theta + dt*x2))/(R + p11 + dt*p21 + (Q*dt^4)/4 + dt*(p12 + dt*p22));
P11 = (R*(4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22))/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22);
P12 = (2*R*(Q*dt^3 + 2*p22*dt + 2*p12))/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22);
P21 = (2*R*(Q*dt^3 + 2*p22*dt + 2*p21))/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22);
P22 = p22 + Q*dt^2 - (((Q*dt^3)/2 + p22*dt + p12)*((Q*dt^3)/2 + p22*dt + p21))/(R + p11 + dt*p21 + (Q*dt^4)/4 + dt*(p12 + dt*p22));
end