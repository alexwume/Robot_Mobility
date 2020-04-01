function [state_dot] = dynamics(params, state, F, M, rpm_motor_dot)
% Input parameters
% 
%   state: current state, will be using ode45 to update
%
%   F, M: actual force and moment from motor model
%
%   rpm_motor_dot: actual change in RPM from motor model
% 
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   state_dot: change in state
%
%************  DYNAMICS ************************

% Write code here
    state_dot=zeros(16,1);
    state_dot(1,:)=state(4); %x_dot
    state_dot(2,:)=state(5); %y_dot
    state_dot(3,:)=state(6); %z_dot
    state_dot(4,:)=F*(cos(state(7))*cos(state(9))*sin(state(8))+sin(state(7))*sin(state(9)))*(1/params.mass); %x_ddot
    state_dot(5,:)=F*(cos(state(7))*sin(state(9))*sin(state(8))-cos(state(9))*sin(state(7)))*(1/params.mass); %y_ddot 
    state_dot(6,:)=(F*cos(state(8))*cos(state(7))-params.mass*params.gravity)*(1/params.mass); %z_ddot
    state_dot(7,:)=state(10); %phi_dot
    state_dot(8,:)=state(11); %theta_dot
    state_dot(9,:)=state(12); %psi_dot
    state_dot(10:12,:) = inv(params.inertia)*(M - cross([state(10);state(11);state(12)],params.inertia*[state(10);state(11);state(12)]));% phi_ddot %theta_ddot %psi_ddot
    state_dot(13:16,:)=rpm_motor_dot; %rpm_motor_dot
    
    
end