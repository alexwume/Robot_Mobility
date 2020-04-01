function [rot, omega] = attitude_planner(desired_state, params)

% Input parameters
%
%   desired_state: The desired states are:
%   desired_state.pos = [x; y; z], 
%   desired_state.vel = [x_dot; y_dot; z_dot],
%   desired_state.rot = [phi; theta; psi], 
%   desired_state.omega = [phidot; thetadot; psidot]
%   desired_state.acc = [xdotdot; ydotdot; zdotdot];
%
%   params: Quadcopter parameters
%
% Output parameters
%
%   rot: will be stored as desired_state.rot = [phi; theta; psi], 
%
%   omega: will be stored as desired_state.omega = [phidot; thetadot; psidot]
%
%************  ATTITUDE PLANNER ************************

% Write code here


psi_d= desired_state.rot(3); %known =0 for hover
psidot=desired_state.omega(3); %known 

%rot=desired_state.rot;% = [phi; theta; psi], 
phi_d=(1/params.gravity)*(sin(psi_d)*desired_state.acc(1)-cos(psi_d)*desired_state.acc(2));
theta_d=(1/params.gravity)*(cos(psi_d)*desired_state.acc(1)+sin(psi_d)*desired_state.acc(2));

rot=[phi_d; theta_d; psi_d];

%omega=desired_state.omega;% = [phidot; thetadot; psidot]
ex_dd=desired_state.acc(1);
ey_dd=desired_state.acc(2);

% xdd=0;
% ydd=0;
% ex_ddd=0;
% ey_ddd=0;
% x_ddd=0;
% y_ddd=0;


%phidot=(1/params.gravity)*(cos(psi_d)*((ex_dd+xdd)*psidot-(ey_ddd+y_ddd))+sin(psi_d)*((ey_dd+ydd)*psidot+(ex_ddd+x_ddd)));
%thetadot=(1/params.gravity)*(-sin(psi_d)*((ex_dd+xdd)*psidot-(ey_ddd+y_ddd))+cos(psi_d)*((ey_dd+ydd)*psidot+(ex_ddd+x_ddd)));

omega = zeros(3,1);
omega(1:2) = 1/params.gravity*[cos(psi_d) sin(psi_d);-sin(psi_d) cos(psi_d)]*[ex_dd*psidot;ey_dd*psidot];
omega(3) = psidot;



end

