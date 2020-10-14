function [sys,x0,str,ts] = timestwo(t,x,u,flag)
%TIMESTWO S-function whose output is two times its input.
%   This M-file illustrates how to construct an M-file S-function that
%   computes an output value based upon its input.  The output of this
%   S-function is two times the input value:
%
%     y = 2 * u;
%
%   See sfuntmpl.m for a general S-function template.
%
%   See also SFUNTMPL.
    
%   Copyright 1990-2002 The MathWorks, Inc.
%   $Revision: 1.7 $

%
% Dispatch the flag. The switch function controls the calls to 
% S-function routines at each simulation stage of the S-function.
%
switch flag,
  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  % Initialize the states, sample times, and state ordering strings.
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  % Return the outputs of the S-function block.
  case 3
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  % There are no termination tasks (flag=9) to be handled.
  % Also, there are no continuous or discrete states,
  % so flags 1,2, and 4 are not used, so return an emptyu
  % matrix 
  case { 1, 2, 4, 9 }
    sys=[];

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Unexpected flags (error handling)%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Return an error message for unhandled flag values.
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end timestwo

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts] = mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 10;  % dynamically sized
sizes.NumInputs      = 8;  % dynamically sized
sizes.DirFeedthrough = 1;   % has direct feedthrough
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
str = [];
x0  = [];
ts  = [1 0];   % inherited sample time

% end mdlInitializeSizes

%
%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================
%
function sys = mdlOutputs(t,x,u)
[Ac, Bc, Cc, Dc, L10, L20, L30, L40, Vp10, Vp20, gamma1, gamma2, g, Kp, kc, Ao1, Ao2, Ao3, Ao4, At1, At2, At3, At4] = Process_setup();
Vmax=22;
umax=3;
m = size(Bc,2); 
n = size(Ac,1); 
rdim=n+m;
% Limit Par
Q = 1*eye(n); 
R = 1*eye(m);
%Define Control Horizon 
Np = 5; % Prediction horizon
Nu = 5; % Control Horizon
T=5; %sampling time in seconds.
J=[Ac Bc; zeros(size(Cc,1),size(Ac,2)) zeros(size(Cc,1),size(Bc,2))];
Jd=expm(T*J);
Ad=Jd(1:size(Ac,1),1:size(Ac,2));
Bd=Jd(1:size(Ac,1),size(Ac,2)+1:end);
Cd=Cc;
Dd=Dc;
P = idare(Ad,Bd,Cd'*Cd,R,[],[]);
[H, F1, F2, F3, Lb, Ub] = condensedmpc_matrix_filler(Ac,Bc,Cc,Dc,Ad, Bd, Cd, Np, Q, R, P, Vp10, Vp20, Vmax);
q=[F1 F3 F2]*u-H*kron(ones((Np),1),[Vp10; Vp20]);
%Ch=[0.4 -0.5; -0.3 0.4];
%Q=[1 0; 0  1];
%Aquad=[1 0;0 1;-1 0; 0 -1];
%bquad=[1; 1; 1; 1];
options=optimset('LargeScale','on','Display','off'); % Switching off the large scale algorithm
    %P=Ch'*Q'*Q*Ch;
    %P=[237.1484 -303.55; -303.55 388.9234];
    %f=-P*u;
    sys=quadprog(H,q,[],[],[],[],kron(ones((Np),1),[0 0]),kron(ones((Np),1),[Vmax Vmax]),[],options);

% end mdlOutputs

