wcw = 200;
wcq = 3750;
Vdc = 220;


Kpw = 0.86
Kiw = 37.21
J = 0.0027
B = 0.0136
Kff = 0.1
T=1e-5
Rs=1.477
Ti=T/2;
Lq=22.5e-3
Ld=19.75e-3
Ls=(Ld+Lq)/2
Lambda_m  = 0.2026
P = 8




syms Kp Ki
eqn1 = wcw*Kp/Ki == tan((75/360)*2*pi - pi/2 + atan(wcw*J/B));
eqn2 = Ki == wcw*sqrt(B^2 +(wcw*J)^2)/sqrt(1+(wcw*Kp/Ki)^2);

[Kiw,Kpw] = solve(eqn1,eqn2);
Kpw = double(Kpw)
Kiw = double(Kiw)

syms Kpq Kiq
eqn3 = wcq*Kpq/Kiq == tan((75/360)*2*pi - pi/2 + atan(wcq*Lq/Rs) + atan(wcq*Ti));
eqn4 = Kiq == wcq*sqrt(Rs^2 + (wcq*Lq)^2)*sqrt(1+(wcq*Ti)^2)/sqrt(1+(wcq*Kpq/Kiq)^2);

[Kiq,Kpq] = solve(eqn3,eqn4);
Kpq = double(Kpq)
Kiq = double(Kiq)

%Kpq = 100
%Kiq = 100

% 
% Q = [0.1 0 0 0;
%     0 0.1 0 0;
%     0 0 15 0;
%     0 0 0 0.0001]
% R = [0.01 0;
%     0 0.01]
% 
% P0=[5 0 0 0;
%     0 5 0 0;
%     0 0 100 0;
%     0 0 0 1]
% 
% omega_prev = 0;
% theta_prev = 0;
% 
% %

%x=x + Fk*x*Ts + B*u*Ts  predicted mean
% estimation error covariance 
%P = P + (Fk*Pk +Pk*Fk^t)*Ts + Q
%Predicted ouptput  : y = Hk*x
%Kalman filter gain calculation :
% Kk = P*Hk^t(Hk*P*Hk^t+R)^-1
%x = x + Kk*(ym-y)
%P = (I - KkHk)*P
