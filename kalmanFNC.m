function [xx1,xx2,pp11,pp12,pp21,pp22] = kalmanFNC(R,Q,dt,theta_k,x1,x2,p11,p12,p21,p22)
xx1 = (4*R*x1 + 4*p11*theta_k + 4*dt^2*p22*theta_k + 4*R*dt*x2 + 4*dt*p12*theta_k + 4*dt*p21*theta_k + Q*dt^4*theta_k)/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22);
xx2 = x2 - (((Q*dt^3)/2 + p22*dt + p21)*(x1 - theta_k + dt*x2))/(R + p11 + dt*p21 + (Q*dt^4)/4 + dt*(p12 + dt*p22));
pp11 = (R*(4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22))/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22);
pp12 = (2*R*(Q*dt^3 + 2*p22*dt + 2*p12))/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22);
pp21 = (2*R*(Q*dt^3 + 2*p22*dt + 2*p21))/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt^4 + 4*dt^2*p22); 
pp22 = p22 + Q*dt^2 - (((Q*dt^3)/2 + p22*dt + p12)*((Q*dt^3)/2 + p22*dt + p21))/(R + p11 + dt*p21 + (Q*dt^4)/4 + dt*(p12 + dt*p22));
end