function [s,t] = QP_SolveTrajectory_setup(T, kc, L10, L20)
%%Trajectory specification
 
 % Set point trajectory including setpoints for u:s
s = [zeros(round(500/T),1); 2*ones(round(960/T),1)]; % This is in "cm" and as deviation from the equilibrium point.
s = [s -s]*kc; %This sets the reference of channel 1 as "s" and for channel 2 as "0"

% Input disturbance trajectory
d = [zeros(round(600/T),1); -0*ones(round(860/T),1)];
d = [zeros(length(d),1) d]*kc; %This sets the disturance for channel 1 as "0" and for channel 2 as "d"
t = 0:T:((size(s,1)-1)*T);
s = s/kc; % Converting the setpoints back to "cm"
s(:,1) = s(:,1)+L10; 
s(:,2) = s(:,2)+L20;
end 
