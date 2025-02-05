% Simulation of a nonlinear quadruple tank lab process controlled by
% an MPC controller using Simulink.
%
% Copyright Johan �kesson 2006
%


clear all
path(path,'../')
% Model for the four tank process

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
h10_nmp=10;
h20_nmp=14;
v20_nmp=Ao1*(1-gamma1)*sqrt(2*g*h10_nmp)/(((1-gamma1)*(1-gamma2)-gamma1*gamma2)*Kp)-Ao2*(gamma1)*sqrt(2*g*h20_nmp)/(((1-gamma1)*(1-gamma2)-gamma1*gamma2)*Kp);
v10_nmp=Ao1*sqrt(2*g*h10_nmp)/(gamma1*Kp)-(1-gamma2)*v20_nmp/gamma1;
h30_nmp=((1-gamma2)*Kp*v20_nmp/Ao3)^2/(2*g);
h40_nmp=((1-gamma1)*Kp*v10_nmp/Ao4)^2/(2*g);

% Build state space model, minimum phase

% Transfer function Time constants
T1_nmp=sqrt(2*h10_nmp/g)*At1/Ao1;
T2_nmp=sqrt(2*h20_nmp/g)*At2/Ao2;
T3_nmp=sqrt(2*h30_nmp/g)*At3/Ao3;
T4_nmp=sqrt(2*h40_nmp/g)*At4/Ao4;

% Trasnfer function coeficients
c1=Kp*Kl*T1_nmp/At1;
c2=Kp*Kl*T1_nmp/At1;
c3=Kp*Kl*T2_nmp/At2;
c4=Kp*Kl*T2_nmp/At2;

A_nmp=[-1/T1_nmp     0     At3/(At1*T3_nmp)     0;
    0       -1/T2_nmp       0          At4/(At2*T4_nmp);
    0        0         -1/T3_nmp        0;
    0        0          0          -1/T4_nmp];
B_nmp=[gamma1*Kp/At1   0;
    0             gamma2*Kp/At2
    0           (1-gamma2)*Kp/At3
    (1-gamma1)*Kp/At4     0];
C_nmp = [kc        0          0           0
     0        kc         0           0]; % Notice the measured signals are given in Volts!
% C_nmp=[1        0          0           0
%     0        1         0           0];
D_nmp =[0         0;     0       0];

h = 5; %sampling time in seconds.

% Constraints
% No constraints on du
% Pump capacities [0 10]V
% Level 1 [0 20]cm = [0 10]V
% Level 2 [0 20]cm
% Level 3 [0 20]cm
% Level 4 [0 20]cm
du_max = [inf inf]'; % limit on delta u; slew rate
du_min = [-inf -inf]';
u_max = [Vmax-v10_nmp Vmax-v20_nmp]'; % limit absolute value of u
u_min = [-v10_nmp -v20_nmp]';
%z_max = [10-h10_nmp/2-0.1 10-h20_nmp/2-0.1 10-h30_nmp/2-0.1 10-h40_nmp/2-0.1]'; % Limits on controlled outputs
%z_min = [-h10_nmp/2 -h20_nmp/2 -h30_nmp/2 -h40_nmp/2]'; 
z_max = [inf inf inf inf]'; % Limits on controlled outputs
z_min = [-inf -inf -inf -inf]'; 


% Set point trajectory including setpoints for u:s
s = [zeros(round(500/h),1); 1*ones(round(960/h),1)]; % This is in "cm" and as deviation from the equilibrium point.
s = [s s]*kc; %This sets the reference of channel 1 as "s" and for channel 2 as "0"

% Input disturbance trajectory
d = [zeros(round(600/h),1); -0*ones(round(860/h),1)];
d = [zeros(length(d),1) d]*kc; %This sets the disturance for channel 1 as "0" and for channel 2 as "d"

% % Set point trajectory
% s = [zeros(50/h,1); 1*ones(round(950/h),1)];
% s = [s zeros(length(s),1)];
% 
% % Input disturbance trajectory
% d = [zeros(600/h,1); -0*ones(round(400/h),1)];
% d = [zeros(length(d),1) d];

% MPC parameters
Hp = 5; % Prediction horizon
Hu = 2; % Horizon for varying input signal
Hw = 1; % First penalty sample
zblk=0;
ublk=1;

% Q = diag([4 1]);
% R = 0.01*diag([1 1]);

Q = diag([1 1]);
R = 1*diag([1 1]);

W = diag([1 1 1 1]);
V = diag(0.01*ones(1,2));

[Ad, Bd, Cyd, Dzd]=ssdata(c2d(ss(A_nmp,B_nmp,C_nmp,D_nmp),h));
Czd = Cyd;


Ccd = kc*eye(4);
Dcd = zeros(4,2);


% md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
% 	    du_max,du_min,u_max,u_min,z_max, ...
% 	    z_min,Q,R,W,V,h,2,'qp_as');
% 
t = 0:h:((size(s,1)-1)*h);

sim_u = [t' s d];

sim_opts = simset;
% 
% [tt,x_internal,xx] = sim('QuadTank_MPC',[0 t(end)],sim_opts,sim_u);
% 
% x = xx(:,1:4);
% u = xx(:,5:6);
% 
% u(:,1) = u(:,1) + v10_nmp;
% u(:,2) = u(:,2) + v20_nmp;
% 
% figure(1);
% clf
% subplot(3,2,1)
% hold on
% stairs(tt,x(:,3),'b--')
% %stairs(tt,kc*x_est(3,:),'--r')
% ylabel('h_3 [cm]')
% %title('h3')
% subplot(3,2,2)
% hold on
% stairs(tt,x(:,4),'b--')
% %stairs(tt,kc*x_est(4,:),'--r')
% ylabel('h_4 [cm]')
% %title('h4')
% subplot(3,2,3)
% hold on
% stairs(tt,x(:,1),'b--')
% %stairs(tt,kc*x_est(1,:),'--r')
% %plot(tt,s(1:max(tt)/h+1,1)','--');
% ylabel('h_1 [cm]')
% %title('h1')
% subplot(3,2,4)
% hold on
% stairs(tt,x(:,2),'b--')
% %stairs(tt,s(1:max(tt)/h+1,2)','--');
% %stairs(tt,kc*x_est(2,:),'--r')
% ylabel('h_2 [cm]')
% %title('h2')
% subplot(3,2,5)
% hold on
% stairs(tt,u(:,1),'b--')
% %title('u1')
% ylabel('u_1 [V]')
% xlabel('t [s]')
% subplot(3,2,6)
% hold on
% stairs(tt,u(:,2),'b--')
% %title('u2')
% ylabel('u_2 [V]')
% xlabel('t [s]')
% zoom on

W = diag([1 1 1 1 1 1]);

md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	    du_max,du_min,u_max,u_min,z_max, ...
	    z_min,Q,R,W,V,h,4,'qp_as'); % This is mode 4 (Disturbance model with error-free tracking)

[tt,x_internal,xx] = sim('QuadTank_MPC',[0 t(end)],sim_opts,sim_u);
x = xx(:,1:4); %Computed Plant states
u = xx(:,5:6); %Computed Plant Inputs

u(:,1) = u(:,1) + v10_nmp; % Actual plant input
u(:,2) = u(:,2) + v20_nmp;

s = s/kc; % Converting the setpoints back to "cm"
s(:,1) = s(:,1)+h10_nmp; 
s(:,2) = s(:,2)+h20_nmp;

figure(2);
subplot(3,2,1)
hold on
stairs(tt/h,x(:,3),'b')
%stairs(tt,kc*x_est(3,:),'--r')
ylabel('h_3 [cm]')
%title('h3')
grid
axis([0 1200/h 0 8])
subplot(3,2,2)
hold on
stairs(tt/h,x(:,4),'b')
%stairs(tt,kc*x_est(4,:),'--r')
ylabel('h_4 [cm]')
%title('h4')
grid
axis([0 1200/h 0 5])
subplot(3,2,3)
hold on
stairs(tt/h,x(:,1),'b')
%stairs(tt,kc*x_est(1,:),'--r')
plot(t/h,s(1:max(tt)/h+1,1)','-.');
ylabel('h_1 [cm]')
%title('h1')
grid
axis([0 1200/h 6 16])
subplot(3,2,4)
hold on
stairs(tt/h,x(:,2),'b')
stairs(t/h,s(1:max(tt)/h+1,2)','-.');
%stairs(tt,kc*x_est(2,:),'--r')
ylabel('h_2 [cm]')
%title('h2')
grid
axis([0 1200/h 5 16])
subplot(3,2,5)
hold on
stairs(tt/h,u(:,1),'b')
%title('u1')
ylabel('u_1 [V]')
xlabel('t [s]')
grid
axis([0 1200/h 0 22])
subplot(3,2,6)
hold on
stairs(tt/h,u(:,2),'b')
grid
%title('u2')
ylabel('u_2 [V]')
xlabel('t [s]')
axis([0 1200/h 2 22])
grid on
%zoom on
  
