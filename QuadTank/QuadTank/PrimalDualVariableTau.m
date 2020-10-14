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

%Control Weights Generation
Q = 1*eye(n); 
R = 1*eye(m);

%Define Control Horizons
Np = 15; % Prediction horizon
Nu = 15; % Control Horizon

% Defining Time Constants
tau1 = input('Enter First Time Constant:    ');
tau2 = input('Enter Second Time Constant:    ');
tau3 = input('Enter Third Time Constant:    ');

tau = tau1;

%% Primal-Dual Solver Set Up.
[H, F1, F2, F3, Lb, Ub, Lx, Ld, T, r, t]=PrimalDual_setup(Ac, Bc, Cc, Dc, Np, Q, R, Vp10, Vp20, Vmax, kc, L10, L20);
% Generate H, E and Fo Matrices 
mdl_tau1 = 'nonlinear_obs_primaldual_2019b';
%open_system(mdl);
%in = Simulink.SimulationInput('nonlinear_obs_primaldual_2019b');
%in = in.setExternalInput('r.getElement(1),r.getElement(2)');
sim_primaldual_tau1 = [t' r];
sim_opts_tau1 = simset;

tic;
out_tau1 = sim(mdl_tau1,[0 t(end)],sim_opts_tau1,sim_primaldual_tau1);
time_1 = toc; 

PrimalDual_t_tau1 = out_tau1.tout;
PrimalDual_out_tau1 = out_tau1.yout{1}.Values;
%PrimalDual_x=Simulink.SimulationData.Dataset(out.yout{1}.Values)
PrimalDual_x_tau1=PrimalDual_out_tau1.data;


%Define Second Time Constant
tau = tau2;

%% Primal-Dual Solver Set Up.
[H, F1, F2, F3, Lb, Ub, Lx, Ld, T, r, t]=PrimalDual_setup(Ac, Bc, Cc, Dc, Np, Q, R, Vp10, Vp20, Vmax, kc, L10, L20);
% Generate H, E and Fo Matrices 
mdl_tau10 = 'nonlinear_obs_primaldual_2019b';
%open_system(mdl);
%in = Simulink.SimulationInput('nonlinear_obs_primaldual_2019b');
%in = in.setExternalInput('r.getElement(1),r.getElement(2)');
sim_primaldual_tau10 = [t' r];
sim_opts_tau10 = simset;
tic;
out_tau10 = sim(mdl_tau10,[0 t(end)],sim_opts_tau10,sim_primaldual_tau10);
time_2 = toc;
PrimalDual_t_tau10 = out_tau10.tout;
PrimalDual_out_tau10 = out_tau10.yout{1}.Values;
%PrimalDual_x=Simulink.SimulationData.Dataset(out.yout{1}.Values)
PrimalDual_x_tau10 = PrimalDual_out_tau10.data;

%Define Third Time Constant
tau = tau3;

%% Primal-Dual Solver Set Up.
[H, F1, F2, F3, Lb, Ub, Lx, Ld, T, r, t]=PrimalDual_setup(Ac, Bc, Cc, Dc, Np, Q, R, Vp10, Vp20, Vmax, kc, L10, L20);
% Generate H, E and Fo Matrices 
mdl_tau100 = 'nonlinear_obs_primaldual_2019b';
%open_system(mdl);
%in = Simulink.SimulationInput('nonlinear_obs_primaldual_2019b');
%in = in.setExternalInput('r.getElement(1),r.getElement(2)');
sim_primaldual_tau100 = [t' r];
sim_opts_tau100 = simset;

tic;
out_tau100 = sim(mdl_tau100,[0 t(end)],sim_opts_tau100,sim_primaldual_tau100);
time_3 = toc; 

PrimalDual_t_tau100 = out_tau100.tout;
PrimalDual_out_tau100 = out_tau100.yout{1}.Values;
%PrimalDual_x=Simulink.SimulationData.Dataset(out.yout{1}.Values)
PrimalDual_x_tau100 = PrimalDual_out_tau100.data;


% Legend 
str1 = strcat('tau =  ', num2str(tau1));
str2 = strcat('tau =  ', num2str(tau2));
str3 = strcat('tau =  ', num2str(tau3));
% Height Plots

subplot(3,2,1)
p=plot(PrimalDual_t_tau1/T, PrimalDual_x_tau1(:,3), 'g', PrimalDual_t_tau10/T, PrimalDual_x_tau10(:,3), 'r',PrimalDual_t_tau100/T, PrimalDual_x_tau100(:,3), 'b')
p(1).LineWidth = 1 
p(2).LineWidth = 1
p(3).LineWidth = 1
ylabel('{x_\xi}_3 [c]')
grid
axis([0 1200/T 0 8])
legend(str1, str2, str3)

subplot(3,2,2)
p=plot(PrimalDual_t_tau1/T, PrimalDual_x_tau1(:,4), 'g', PrimalDual_t_tau10/T, PrimalDual_x_tau10(:,4), 'r',PrimalDual_t_tau100/T, PrimalDual_x_tau100(:,4), 'b')
p(1).LineWidth = 1 
p(2).LineWidth = 1
p(3).LineWidth = 1
ylabel('{x_\xi}_4 [c]')
grid
axis([0 1200/T 0 5])
legend(str1, str2, str3)

subplot(3,2,4)
hold on
p=plot(PrimalDual_t_tau1/T, PrimalDual_x_tau1(:,2), 'g', PrimalDual_t_tau10/T, PrimalDual_x_tau10(:,2), 'r',PrimalDual_t_tau100/T, PrimalDual_x_tau100(:,2), 'b')
p(1).LineWidth = 1 
p(2).LineWidth = 1
p(3).LineWidth = 1
z=plot(t/T,r(1:max(PrimalDual_t_tau1)/T+1,2)','-.');
hold off
grid
axis([0 1200/T 0 16])
ylabel('{x_\xi}_2 [c]')
legend(str1, str2, str3)

subplot(3,2,3)
hold on
p=plot(PrimalDual_t_tau1/T, PrimalDual_x_tau1(:,1), 'g', PrimalDual_t_tau10/T, PrimalDual_x_tau10(:,1), 'r',PrimalDual_t_tau100/T, PrimalDual_x_tau100(:,1), 'b')
p(1).LineWidth = 1 
p(2).LineWidth = 1
p(3).LineWidth = 1
z=plot(t/T,r(1:max(PrimalDual_t_tau1)/T+1,1)','-.');
hold off
grid
axis([0 1200/T 6 16])
ylabel('{x_\xi}_1 [c]')
legend(str1, str2, str3)

subplot(3,2,5)
p=plot(PrimalDual_t_tau1/T, PrimalDual_x_tau1(:,5), 'g', PrimalDual_t_tau10/T, PrimalDual_x_tau10(:,5), 'r',PrimalDual_t_tau100/T, PrimalDual_x_tau100(:,5), 'b')
p(1).LineWidth = 1 
p(2).LineWidth = 1
p(3).LineWidth = 1
xlabel('t [s]')
ylabel('{u_\xi}_1 [c]')
grid
axis([0 1200/T 0 22])
legend(str1, str2, str3)

subplot(3,2,6)
p=plot(PrimalDual_t_tau1/T, PrimalDual_x_tau1(:,6), 'g', PrimalDual_t_tau10/T, PrimalDual_x_tau10(:,6), 'r',PrimalDual_t_tau100/T, PrimalDual_x_tau100(:,6), 'b')
p(1).LineWidth = 1 
p(2).LineWidth = 1
p(3).LineWidth = 1
xlabel('t [s]')
ylabel('{u_\xi}_2 [c]')
axis([0 1200/T 2 22])
grid on
legend(str1, str2, str3)
