
function [md, s, d, t, h] = MPCTool_setup(Ac, Bc, Cc, Dc, Np, Q, R, Vp10, Vp20, Vmax, kc)

%% ----------------- MPC Data------------------------
m = size(Bc,2); 
n = size(Ac,1); 
Hp=Np; %Prediction horizon
Hu=Np; % Control horizon
Hw = 1; % First penalty sample
Q = diag([1 1]);
zblk=0;
ublk=1;
v10_nmp=Vp10;
v20_nmp=Vp20;

% Constraint Specifications
du_max = [inf inf]'; % limit on delta u; slew rate
du_min = [-inf -inf]';
u_max = [Vmax-v10_nmp Vmax-v20_nmp]'; % limit absolute value of u
u_min = [-v10_nmp -v20_nmp]';
%z_max = [10-h10_nmp/2-0.1 10-h20_nmp/2-0.1 10-h30_nmp/2-0.1 10-h40_nmp/2-0.1]'; % Limits on controlled outputs
%z_min = [-h10_nmp/2 -h20_nmp/2 -h30_nmp/2 -h40_nmp/2]'; 
z_max = [inf inf inf inf]'; % Limits on controlled outputs
z_min = [-inf -inf -inf -inf]'; 

% Discretized State-Space Model 

% Using the most accurate Method
h=5;  %sampling time in seconds.
[Ad, Bd, Cyd, Dzd]=ssdata(c2d(ss(Ac,Bc,Cc,Dc),h));
Czd = Cyd;

V = diag(0.01*ones(1,2));
W = diag([1 1 1 1 1 1]);
Ccd = kc*eye(4);
Dcd = zeros(4,2);


md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	    du_max,du_min,u_max,u_min,z_max, ...
	    z_min,Q,R,W,V,h,4,'qp_as'); % This is mode 4 (Disturbance model with error-free tracking)
    
 %%Trajectory specification
 
 % Set point trajectory including setpoints for u:s
s = [zeros(round(500/h),1); 2*ones(round(960/h),1)]; % This is in "cm" and as deviation from the equilibrium point.
s = [s -s]*kc; %This sets the reference of channel 1 as "s" and for channel 2 as "0"

% Input disturbance trajectory
d = [zeros(round(600/h),1); -0*ones(round(860/h),1)];
d = [zeros(length(d),1) d]*kc; %This sets the disturance for channel 1 as "0" and for channel 2 as "d"
t = 0:h:((size(s,1)-1)*h);

end 
