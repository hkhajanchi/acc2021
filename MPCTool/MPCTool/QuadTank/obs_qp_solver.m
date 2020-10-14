function [sys,x0,str,ts] = obs_qp_solver(t,x,u,flag,dd)
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
switch flag
  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  % Initialize the states, sample times, and state ordering strings.
  case 0
     [sys,x0,str,ts] = mdlInitializeSizes(dd);
  % Updates
     case 2
     sys = mdlUpdates(t,x,u,dd); % Update discrete states

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  % Return the outputs of the S-function block.
  case 3
    sys = mdlOutputs(t,x,u,dd);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  % There are no termination tasks (flag=9) to be handled.
  % Also, there are no continuous or discrete states,
  % so flags 1,2, and 4 are not used, so return an emptyu
  % matrix 
  case {1, 4, 9}
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
function [sys,x0,str,ts] = mdlInitializeSizes(dd)

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = dd.n_states + dd.n_outputs+ dd.n_inputs + dd.n_outputs;
sizes.NumOutputs     = dd.n_inputs + dd.n_states;  % dynamically sized
sizes.NumInputs      = dd.n_outputs + dd.n_outputs;  % dynamically sized
sizes.DirFeedthrough = 1;   % has direct feedthrough
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(dd.n_states + dd.n_outputs + dd.n_inputs +  dd.n_outputs,1);   

global x;
x = zeros(dd.n_states + dd.n_outputs + dd.n_inputs  + dd.n_outputs,1); 
str = [];
ts  = [dd.T 0];   % inherited sample time

% end mdlInitializeSizes

%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(~,x,~,~)

sys =  x;
% End of mdlUpdate.

%
%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================
%
function sys = mdlOutputs(t,x_i,u,dd)
global x;

fprintf('Update start, t=%4.2f\n',t)
%fprintf('Update start, u=%4.2f\n',u)

x_hat = x(1:dd.n_states,:);
d_hat = x(dd.n_states+1:dd.n_states+dd.n_outputs,:);
u_old = x(dd.n_states+dd.n_outputs+1:dd.n_states+dd.n_outputs+dd.n_inputs,:);
y_old = x(dd.n_states+dd.n_outputs+dd.n_inputs+1:dd.n_states+dd.n_outputs+dd.n_inputs+dd.n_outputs,:);
r = u(1:dd.n_outputs)-[dd.L10 dd.L20]';
y_plant = u(dd.n_outputs+1:dd.n_outputs+dd.n_outputs)-[dd.L10 dd.L20]';

% Update observer
  x_hat = dd.Ad*x_hat + dd.Bd*u_old + dd.Lx*(y_old - dd.Cd*x_hat-d_hat);
  d_hat = d_hat + dd.Ldd*(y_old - dd.Cd*x_hat-d_hat);
  
  y_old = y_plant;
  
% Optimize
%q=[F1 F3 F2]*u-H*kron(ones((Np),1),[Vp10; Vp20]);
q=dd.F1*x_hat+dd.F2*d_hat+dd.F3*r;

options=optimset('LargeScale','on','Display','off'); % Switching off the large scale algorithm
U=quadprog(dd.H,q,[],[],[],[],dd.Lb,dd.Ub,[],options);
u_old=u_old-dd.T*[u_old-[eye(dd.n_inputs) zeros(dd.n_inputs, dd.n_inputs*(dd.Np-1))]*U]/25;
%v_old=u_old+[dd.Vp10 dd.Vp20]';
%updates

x(1:dd.n_states,:) = x_hat;
x(dd.n_states+1:dd.n_states+dd.n_outputs,:)=d_hat;
x(dd.n_states+dd.n_outputs+1:dd.n_states+dd.n_outputs+dd.n_inputs,:) = u_old;
x(dd.n_states+dd.n_outputs+dd.n_inputs+1:dd.n_states+dd.n_outputs+dd.n_inputs+dd.n_outputs,:) = y_old;
sys=[x(dd.n_states+dd.n_outputs+1:dd.n_states+dd.n_outputs+dd.n_inputs,:);  x(1:dd.n_states,:)];

% end mdlOutputs

