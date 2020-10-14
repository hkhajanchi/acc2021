% Dr. Ambrose Adegbege
% The College of New Jersey
% Department of Electrical and Computer Engineering
%
%-------------------------------------------------------------------------
% Model Predictive Control (MPC) Problem Matrix Filler.
%-------------------------------------------------------------------------

%-----------------------------------------------------
function [H, F1, F2, F3, Lb, Ub] = condensedmpc_matrix_filler(Ac,Bc,Cc,Dc,Ad, Bd, Cd, Np, Q, R, P, Vp10, Vp20, Vmax)

    %------------------------------------------------
    % Construct model
    %------------------------------------------------
    % Specify system here in state space form with A and B
    %Ap=[-0.01 0;0.0 -0.01]; Bp=[0.4 -0.5; -0.3  0.4]; %plant
    %Ap=[0 1; -1 0]; Bp=[0; 1]; 
    % Specify system dimensions
    n_inputs = size(Bd,2); % # inputs
    n_states = size(Ad,1); % # states

    %-------------------------------------------------
    % Specify horizons
    %------------------------------------------------
    %Nu = 1; % Control Horizon
    %N = 30; % Prediction Horizon

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
end