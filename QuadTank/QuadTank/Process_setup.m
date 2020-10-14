%Script that returns continuous time A,B,C,D State Space Matrices
function [Ac, Bc, Cc, Dc, L10, L20, L30, L40, Vp10, Vp20, gamma1, gamma2, g, Kp, kc, Ao1, Ao2, Ao3, Ao4, At1, At2, At3, At4] = Process_setup()

% Process parameters
% Kp    Pump Flow Constant                      (cm^3/s/V)
Kp=3.3;
k1_nmp=3.3; % cm^3/Vs
k2_nmp=3.3; % cm^3/Vs
% Kl    Tank Water Level Sensor Sensitivity   (cm/V)
Kl=25/4.15;
kc=4.15/25;
% Gravitational Constant on Earth (cm/s^2)
g = 981;

% Maximum voltage to the pump (V)

Vmax=22;

% Maximum voltage from FPAA board (v)

umax=3;

% Scaling factor for control optimization

alpha=Vmax/umax;

% Specifying TanK/inlet/Outlet Diameters%-----------------------
% Tank 1 inside diameter (cm)
Dt1=4.445;
Dt2=Dt1;
Dt3=Dt1;
Dt4=Dt1;
Dmo=0.47625;
Do1=Dmo;
Do2=Dmo;
Do3=Dmo;
Do4=Dmo;
Dout1=0.635;
Dout2=0.47625;
Di1=Dout1;
Di2=Dout1;
Di3=Dout2;
Di4=Dout2;

 
% Calculate the system's areas
% Tank 1 Inside Cross-Section Area (cm^2)
At1 = pi * Dt1^2 / 4; A1=At1;
% Tank 2 Inside Cross-Section Area (cm^2)
At2 = pi * Dt2^2 / 4; A2=At2;
% Tank 3 Inside Cross-Section Area (cm^2)
At3 = pi * Dt3^2 / 4; A3=At3;
% Tank 4 Inside Cross-Section Area (cm^2)
At4 = pi * Dt4^2 / 4; A4=At4;
% Tank 1 Outlet Area (cm^2)
Ao1 = pi * Do1^2 / 4; a1=Ao1;
% Tank 2 Outlet Area (cm^2)
Ao2 = pi * Do2^2 / 4 ; a2=Ao2;
% Tank 3 Outlet Area (cm^2)
Ao3 = pi * Do3^2 / 4; a3=Ao3;
% Tank 4 Outlet Area (cm^2)
Ao4 = pi * Do4^2 / 4; a4=Ao4;
% Tank 1 Inlet Area (cm^2)
Ai1 = pi * Di1^2 / 4;
% Tank 2 Inlet Area (cm^2)
Ai2 = pi * Di2^2 / 4;
% Tank 3 Inlet Area (cm^2)
Ai3 = pi * Di3^2 / 4;
% Tank 4 Inlet Area (cm^2)
Ai4 = pi * Di4^2 / 4;

%Calculating Flow Split Coeficients
gamma1=Ai1/(Ai1+Ai4);
gamma2=Ai2/(Ai2+Ai3);
g1_nmp=gamma1;
g2_nmp=gamma2;

% If symbolic toolbox is installed, this code could be used to
% calculate the stationary values
% Stationary values, non-minimum phase
%S = solve('-a1/A1*sqrt(2*g*h10_nmp)+a3/A1*sqrt(2*g*h30_nmp)+g1_nmp*k1_nmp/A1*v10_nmp',...
%	  '-a2/A2*sqrt(2*g*h20_nmp)+a4/A2*sqrt(2*g*h40_nmp)+g2_nmp*k2_nmp/A2*v20_nmp',...
%	  '-a3/A3*sqrt(2*g*h30_nmp)+(1-g2_nmp)*k2_nmp/A3*v20_nmp',...
%	  '-a4/A4*sqrt(2*g*h40_nmp)+(1-g1_nmp)*k1_nmp/A4*v10_nmp',...
%	  'h10_nmp,h20_nmp,h30_nmp,h40_nmp');

%h10_nmp = eval(S.h10_nmp);
%h20_nmp = eval(S.h20_nmp);
%h30_nmp = eval(S.h30_nmp);
%h40_nmp = eval(S.h40_nmp);

%Equilibrium Calculations:
L10=10;
L20=14;
Vp20=Ao1*(1-gamma1)*sqrt(2*g*L10)/(((1-gamma1)*(1-gamma2)-gamma1*gamma2)*Kp)-Ao2*(gamma1)*sqrt(2*g*L20)/(((1-gamma1)*(1-gamma2)-gamma1*gamma2)*Kp);
Vp10=Ao1*sqrt(2*g*L10)/(gamma1*Kp)-(1-gamma2)*Vp20/gamma1;
L30=((1-gamma2)*Kp*Vp20/Ao3)^2/(2*g);
L40=((1-gamma1)*Kp*Vp10/Ao4)^2/(2*g);

% Build state space model, minimum phase

% Transfer function Time constants
T1_nmp=sqrt(2*L10/g)*At1/Ao1;
T2_nmp=sqrt(2*L20/g)*At2/Ao2;
T3_nmp=sqrt(2*L30/g)*At3/Ao3;
T4_nmp=sqrt(2*L40/g)*At4/Ao4;

% Trasnfer function coeficients
c1=Kp*Kl*T1_nmp/At1;
c2=Kp*Kl*T1_nmp/At1;
c3=Kp*Kl*T2_nmp/At2;
c4=Kp*Kl*T2_nmp/At2;

Ac=[-1/T1_nmp     0     At3/(At1*T3_nmp)     0;
    0       -1/T2_nmp       0          At4/(At2*T4_nmp);
    0        0         -1/T3_nmp        0;
    0        0          0          -1/T4_nmp];
Bc=[gamma1*Kp/At1   0;
    0             gamma2*Kp/At2
    0           (1-gamma2)*Kp/At3
    (1-gamma1)*Kp/At4     0];
Cc = [kc        0          0           0
     0        kc         0           0]; % Notice the measured signals are given in Volts!
% C_nmp=[1        0          0           0
%     0        1         0           0];
Dc =[0         0;     0       0];


end 
