clear all
%% Generate A,B,C Matrices based on Quad-Tank system
[Ac, Bc, Cc, Dc, L10, L20, L30, L40, Vp10, Vp20, gamma1, gamma2, g, Kp, kc, Ao1, Ao2, Ao3, Ao4, At1, At2, At3, At4] = Process_setup(); %Generates A,B,C,D matrixes of CT system
 %Cc=eye(2,4); Cd=Cc;
%% Common MPC Parameters.
%Size Parameters
m = size(Bc,2); 
n = size(Ac,1); 
rdim=n+m;
% Limit Parameters
Vmax=22;
umax=3;
alpha=Vmax/umax; % Scaling factor for control optimization
%Define Control Horizon 
Np = 5; % Prediction horizon
Nu = 1; % Control Horizon

%Control Weights Generation
Q = 1*eye(n); 
R = 1*eye(m);

%% Primal-Dual Solver Set Up.
[H, F1, F2, F3, Lb, Ub, Lx, Ld, T, r, t]=PrimalDual_setup(Ac, Bc, Cc, Dc, Np, Q, R, Vp10, Vp20, Vmax, kc, L10, L20);
% Generate H, E and Fo Matrices 
mdl = 'nonlinear_obs_primaldual_2019b';
%open_system(mdl);
%in = Simulink.SimulationInput('nonlinear_obs_primaldual_2019b');
%in = in.setExternalInput('r.getElement(1),r.getElement(2)');
sim_primaldual = [t' r];
sim_opts = simset;
out = sim(mdl,[0 t(end)],sim_opts,sim_primaldual);
PrimalDual_t = out.tout;
PrimalDual_out = out.yout{1}.Values;
%PrimalDual_x=Simulink.SimulationData.Dataset(out.yout{1}.Values)
PrimalDual_x=PrimalDual_out.data;


figure(1);
subplot(3,2,1)
hold on
stairs(PrimalDual_t/T,PrimalDual_x(:,3),'b')
%stairs(tt,kc*x_est(3,:),'--r')
ylabel('h_3 [cm]')
%title('h3')
grid
axis([0 1200/T 0 8])
subplot(3,2,2)
hold on
stairs(PrimalDual_t/T,PrimalDual_x(:,4),'b')
%stairs(tt,kc*x_est(4,:),'--r')
ylabel('h_4 [cm]')
%title('h4')
grid
axis([0 1200/T 0 5])
subplot(3,2,3)
hold on
stairs(PrimalDual_t/T,PrimalDual_x(:,1),'b')
%stairs(tt,kc*x_est(1,:),'--r')
plot(t/T,r(1:max(PrimalDual_t)/T+1,1)','-.');
ylabel('h_1 [cm]')
%title('h1')
grid
axis([0 1200/T 6 16])
subplot(3,2,4)
hold on
stairs(PrimalDual_t/T,PrimalDual_x(:,2),'b')
plot(t/T,r(1:max(PrimalDual_t)/T+1,2)','-.');
%stairs(tt,kc*x_est(2,:),'--r')
ylabel('h_2 [cm]')
%title('h2')
grid
axis([0 1200/T 0 16])
subplot(3,2,5)
hold on
stairs(PrimalDual_t/T,PrimalDual_x(:,5),'b')
%title('u1')
ylabel('u_1 [V]')
xlabel('t [s]')
grid
axis([0 1200/T 0 22])
subplot(3,2,6)
hold on
stairs(PrimalDual_t/T,PrimalDual_x(:,6),'b')
grid
%title('u2')
ylabel('u_2 [V]')
xlabel('t [s]')
axis([0 1200/T 2 22])
grid on
%zoom on

 
 %% MPC-Tool Solver Set Up.
 [md, s, d, t, h]=MPCTool_setup(Ac, Bc, Cc, Dc, Np, Q, R, Vp10, Vp20, Vmax, kc);
 
g1_nmp=gamma1;
g2_nmp=gamma2;
k1_nmp=Kp; k2_nmp=Kp;
h10_nmp=L10; h20_nmp=L20; h30_nmp=L30; h40_nmp=L40;
A1=At1; A2=At2; A3=At3; A4=At4;
a1=Ao1; a2=Ao2; a3=Ao3; a4=Ao4;
v10_nmp=Vp10; v20_nmp=Vp20;
sim_u = [t' s d];
sim_opts = simset;
[tt,x_internal,xx]= sim('QuadTank_MPC',[0 t(end)],sim_opts,sim_u);
% tt = simout.tout;
% tt_out = simout.yout{1}.Values;
% xx=tt_out.data;
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
%% PI Controller Set up
 [lambda1, lambda2, r_pi]=PI_setup(T, kc, L10, L20);
 
 mdl = 'NonlinearTank_PI_2019b';
%open_system(mdl);
%in = Simulink.SimulationInput('nonlinear_obs_primaldual_2019b');
%in = in.setExternalInput('r.getElement(1),r.getElement(2)');
sim_pi = [t' r_pi];
sim_opts = simset;
out = sim(mdl,[0 t(end)],sim_opts,sim_pi);
PI_t = out.tout;
PI_out = out.yout{1}.Values;
%PrimalDual_x=Simulink.SimulationData.Dataset(out.yout{1}.Values)
PI_x=PI_out.data;


figure(3);
subplot(3,2,1)
hold on
stairs(PI_t/T,PI_x(:,3),'b')
%stairs(tt,kc*x_est(3,:),'--r')
ylabel('h_3 [cm]')
%title('h3')
grid
axis([0 1200/T 0 8])
subplot(3,2,2)
hold on
stairs(PI_t/T,PI_x(:,4),'b')
%stairs(tt,kc*x_est(4,:),'--r')
ylabel('h_4 [cm]')
%title('h4')
grid
axis([0 1200/T 0 5])
subplot(3,2,3)
hold on
stairs(PI_t/T,PI_x(:,1),'b')
%stairs(tt,kc*x_est(1,:),'--r')
plot(t/T,r_pi(1:max(PI_t)/T+1,1)','-.');
ylabel('h_1 [cm]')
%title('h1')
grid
axis([0 1200/T 6 16])
subplot(3,2,4)
hold on
stairs(PI_t/T,PI_x(:,2),'b')
plot(t/T,r_pi(1:max(PI_t)/T+1,2)','-.');
%stairs(tt,kc*x_est(2,:),'--r')
ylabel('h_2 [cm]')
%title('h2')
grid
axis([0 1200/T 0 16])
subplot(3,2,5)
hold on
stairs(PI_t/T,PI_x(:,5),'b')
%title('u1')
ylabel('u_1 [V]')
xlabel('t [s]')
grid
axis([0 1200/T 0 22])
subplot(3,2,6)
hold on
stairs(PI_t/T,PI_x(:,6),'b')
grid
%title('u2')
ylabel('u_2 [V]')
xlabel('t [s]')
axis([0 1200/T 2 22])
grid on
%zoom on

%% QuadProg Controller Set up
[dd] = Process_data();
mdl = 'nonlinear_qp_solver_2019b';
%open_system(mdl);
%in = Simulink.SimulationInput('nonlinear_obs_primaldual_2019b');
%in = in.setExternalInput('r.getElement(1),r.getElement(2)');
[r_qu,t] = QP_SolveTrajectory_setup(T, kc, L10, L20);
sim_qu = [t' r_qu];
sim_opts = simset;
out= sim(mdl,[0 t(end)],sim_opts,sim_qu);
%out=sim(mdl)
QuadProg_t = out.tout;
QuadProg_out = out.yout{1}.Values;
%PrimalDual_x=Simulink.SimulationData.Dataset(out.yout{1}.Values)
QuadProg_x=QuadProg_out.data;

figure(4);
subplot(3,2,1)
hold on
stairs(QuadProg_t/T,QuadProg_x(:,3),'b')
%stairs(tt,kc*x_est(3,:),'--r')
ylabel('h_3 [cm]')
%title('h3')
grid
axis([0 1200/T 0 8])
subplot(3,2,2)
hold on
stairs(QuadProg_t/T,QuadProg_x(:,4),'b')
%stairs(tt,kc*x_est(4,:),'--r')
ylabel('h_4 [cm]')
%title('h4')
grid
axis([0 1200/T 0 5])
subplot(3,2,3)
hold on
stairs(QuadProg_t/T,QuadProg_x(:,1),'b')
%stairs(tt,kc*x_est(1,:),'--r')
plot(t/T,r_qu(1:max(QuadProg_t)/T+1,1)','-.');
ylabel('h_1 [cm]')
%title('h1')
grid
axis([0 1200/T 6 16])
subplot(3,2,4)
hold on
stairs(QuadProg_t/T,QuadProg_x(:,2),'b')
plot(t/T,r_qu(1:max(QuadProg_t)/T+1,2)','-.');
%stairs(tt,kc*x_est(2,:),'--r')
ylabel('h_2 [cm]')
%title('h2')
grid
axis([0 1200/T 0 20])
subplot(3,2,5)
hold on
stairs(QuadProg_t/T,QuadProg_x(:,5),'b')
%title('u1')
ylabel('u_1 [V]')
xlabel('t [s]')
grid
axis([0 1200/T 0 22])
subplot(3,2,6)
hold on
stairs(QuadProg_t/T,QuadProg_x(:,6),'b')
grid
%title('u2')
ylabel('u_2 [V]')
xlabel('t [s]')
axis([0 1200/T 2 22])
grid on
%zoom on
