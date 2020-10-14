%Script that returns continuous time A,B,C,D State Space Matrices
function dd = Process_data()

global dd;
dd = struct('Ad',[],... % Discrete system matrix,
	    'Bd',[],... % Discrete input matrix,
	    'Cd',[],... % Discrete output matrix,
	    'Dd',[],... % Discrete feedthrough matrix
	    'Lx',[],... % Observer gain, states
	    'Ldd',[],... % Observer gain, disturbances
	    'Vp10',[],... % Operating point, channel one input voltage 
	    'Vp20',[],... % Operating point, channel two input voltage
        'L10',[],... % Operating point, channel one level 
	    'L20',[],... % Operating point, channel two level
        'L30',[],... % Operating point, channel one level 
	    'L40',[],... % Operating point, channel two level
	    'H',[],... % Hessian of QP problem
	    'F1',[],... % Input to QP problem 
	    'F2',[],... % Input to QP problem
	    'F3',[],... %Input to QP problem
	    'Lb',[],... % Constraints
	    'Ub',[],... % Constraints 
	    'Aob',[],... % A matrix for extended discretized system
	    'Bob',[],... % B matrix for extended discretized system
	    'Cob',[],... % C matrix for extended discretized system
	    'Dob',[],... % D matrix for extended discretized system
	    'n_states',[],... % Number of states for system
	    'n_inputs',[],... % Number of inputs for system
        'n_outputs',[],... % Number of outputs for system
	    'Np',[],... % Control signal horizon
        'kc',[],... % level conversion V/cm
	    'T',[]); % The number of explicit integrators

% Process parameters
% Kp    Pump Flow Constant                      (cm^3/s/V)
Kp=3.3;
k1_nmp=3.3; % cm^3/Vs
k2_nmp=3.3; % cm^3/Vs
% Kl    Tank Water Level Sensor Sensitivity   (cm/V)
Kl=25/4.15;
kc=4.15/25;
dd.kc=kc;
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

%Equilibrium Calculations:
L10=10;
L20=14;
Vp20=Ao1*(1-gamma1)*sqrt(2*g*L10)/(((1-gamma1)*(1-gamma2)-gamma1*gamma2)*Kp)-Ao2*(gamma1)*sqrt(2*g*L20)/(((1-gamma1)*(1-gamma2)-gamma1*gamma2)*Kp);
Vp10=Ao1*sqrt(2*g*L10)/(gamma1*Kp)-(1-gamma2)*Vp20/gamma1;
L30=((1-gamma2)*Kp*Vp20/Ao3)^2/(2*g);
L40=((1-gamma1)*Kp*Vp10/Ao4)^2/(2*g);

dd.L10=L10;
dd.L20=L20;
dd.L30=L30;
dd.L40=L40;
dd.Vp10=Vp10;
dd.Vp20=Vp20;
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
%Cc = [kc        0          0           0
%     0        kc         0           0]; % Notice the measured signals are given in Volts!
Cc=[1        0          0           0
     0        1         0           0];
Dc =[0         0;     0       0];

%%--------Discretization---------------------------------------
T=5; %sampling time
 
J=[Ac Bc; zeros(size(Cc,1),size(Ac,2)) zeros(size(Cc,1),size(Bc,2))];
Jd=expm(T*J);
Ad=Jd(1:size(Ac,1),1:size(Ac,2));
Bd=Jd(1:size(Ac,1),size(Ac,2)+1:end);
Cd=Cc;
Dd=Dc;
dd.T=T;
dd.Ad=Ad;
dd.Bd=Bd;
dd.Cd=Cd;
dd.Dd=Dd;
%% ----------------- Intialize Observer ------------------------
n_inputs = size(Bd,2); % # inputs
n_outputs = size(Cd,1); % # inputs
n_states = size(Ad,1); % # states

dd.n_inputs=n_inputs;
dd.n_outputs=n_outputs;
dd.n_states=n_states;

Aob=[Ad zeros( n_states,n_inputs); zeros(n_inputs, n_states) eye(n_inputs)]; %Augmented system matrix
Bob=[Bd; zeros(n_inputs,n_inputs)]; % Augmented Input Matrix
Cob=[Cd eye(n_inputs)]; % Augmented Output Matrix
Dob=Dd; %Augmented Feedthrough Matrix

dd.Aob=Aob;
dd.Bob=Bob;
dd.Cob=Cob;
dd.Dob=Dob;
%----------------------------------------------------------------------
% Calculating the observer gain using "place" 
 Ld = place(Aob',Cob',[0.1 0.5 0.3 0.6 0.3 0.5]');
 Ld=Ld';%observer gain with Ld=[Lx ; Ldd]
 Lx=Ld(1:n_states,:);
 Ldd=Ld(n_states+1:end,:);
 
 dd.Lx=Lx;
 dd.Ldd=Ldd;
 
 %%-------MPC Data Generation-------------
  %-------------------------------------------------
    % Specify horizons
    %------------------------------------------------
    %Nu = 1; % Control Horizon
    Np = 5; % Prediction Horizon
    %------------------------------------------------------
    %Control Weights Generation
    Q = 1*eye(n_states); 
    R = 1*eye(n_inputs); 
    P = idare(Ad,Bd,Cd'*Cd,[],[],[]);
    
    dd.Np=Np;

    %------------------------------------------------------
    % Initial Matrix Filling
    %------------------------------------------------------
    Phi = zeros(Np*n_states,n_states);
    Psi = zeros(Np*n_states,n_inputs);
    Lambda = zeros(Np*n_states,Np*n_inputs);
    % Input values for Phi 
    for i = 1:Np,
        Phi(1+(i-1)*n_states:i*n_states,:) = Ad^i;
    end;
    % Input values for Psi 
    Psi(1:n_states,:)=Ad^0 *eye(n_states,n_inputs);
    for i = 2:Np,
        Psi(1+(i-1)*n_states:i*n_states,:) = (Ad^(i-1)*eye(n_states,n_inputs))+ Psi(1+(i-2)*n_states:(i-1)*n_states,:);
    end;
    % Input values for Lambda
    AB=zeros(Np*n_states,n_inputs);
    AB(1:n_states,:)=Bd;
    for i = 2:Np,
        AB((i-1)*n_states+1:(i)*n_states,:)=Ad^(i-1)*Bd;
    end;
    for i = 1:Np,
        Lambda(1+(i-1)*n_states:end,1+(i-1)*n_inputs:(i)*n_inputs)=AB(1:(Np-i+1)*n_states,:);
    end;
    
%------------------------------------------------
% Quadratic cost weighting Q (state) and R (control) generation.
%------------------------------------------------
Q = sparse(blkdiag(kron(eye(Np-1),Q),P));
R = sparse(kron(eye(Np),R));
%-----------------------------------------------
% Generate M and F1
%-----------------------------------------------
H = 0.5*full(Lambda'*Q*Lambda + R);
F1 = Lambda'*Q*Phi;
%----------------------------------------------
%Computing Steady-State Target Matrices
%----------------------------------------------
I=[Ad-eye(n_states) Bd; Cd zeros(n_inputs)]; %steady state matrix
%I=[Ac Bc; Cc zeros(n_inputs)]; %steady state matrix
IV=inv(I); % Inverse of the steady-state matrix
% Extracting Mt and Nt
Mt=IV(1:n_states,n_states+1:end);
Nt=IV(n_states+1:end,n_states+1:end);

M = zeros(Np*n_states,n_inputs);
N = zeros(Np*n_inputs,n_inputs);
    for i = 1:Np,
        M(1+(i-1)*n_states:i*n_states,:) = Mt;
        N(1+(i-1)*n_inputs:i*n_inputs,:) = Nt;
    end;
%-----------------------------------------------
% Generate F2 and F3
%-----------------------------------------------
F2 = Lambda'*Q*M +R*N; %Assuming disturbance only affect the output and not the states.
F3 = -Lambda'*Q*M-R*N;
%-----------------------------------------------
% Create plant constraints
%-----------------------------------------------
% lb = -3.3*ones(n_inputs,1); 
% ub = 3.3*ones(n_inputs,1);
%Vmax=22;
lb(1) = -Vp10; lb(2)=-Vp20;
ub(1) = Vmax-Vp10; ub(2)=Vmax-Vp20;
Lb = kron(ones((Np),1),lb'); 
Ub = kron(ones((Np),1),ub');

dd.H=H;
dd.F1=F1;
dd.F2=F2;
dd.F3=F3;
dd.Lb=Lb;
dd.Ub=Ub;

end 
