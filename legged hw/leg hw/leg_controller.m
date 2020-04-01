clear all
close all
%model parameters
m=80;
r=0.05;
k=20000;
Jm=0.000506;
Tau_max=1.313;
N=40;
g=9.81;
%PD control gains
Kp=8000; 
Kd=1000; 
Ki=4500;
Kp_s=10; 
Kd_s=5; 
Ki_s=0;


%initial conditions
ts=0:0.01:10;
xo=[1;0;0;0;0;0];

%desired height
for ydes=0.7:0.1:0.9
yo=1;
[t,x,Tau]=ode45(@(t,x) ODE(r,k,Jm,m,Tau_max,N,g,Kp,Kd,Ki,Kp_s,Kd_s,Ki_s,ydes,yo,t,x),ts,xo);
figure
plot(t,x(:,1))
title(['y position over time at ydes=', num2str(ydes)]);
xlabel('time');
ylabel('y (m)');

%motor_torque
y=x(:,1);
y_dot=x(:,2);
y_int=x(:,5);
theta=x(:,3);
theta_dot=x(:,4);
theta_int=x(:,6);
y_er=ydes-x(:,1);
y_ddot=-g+k*(yo-y+r*theta/N)/m;
F_des=Kp*y_er+Kd*(-x(:,2))+Ki*y_int;
F_des_dot=Kp*(-x(:,2))+Kd*-y_ddot+Ki*(ydes-x(:,1));
theta_des=(F_des/k+x(:,1)-yo)*N/r;
theta_des_dot=(F_des_dot/k+x(:,2))*N/r;
theta_er=theta_des-x(:,3);
Tau=Kp_s*theta_er+Kd_s*(theta_des_dot-x(:,4))+Ki_s*theta_int;

Tau=sign(Tau).*min(Tau_max,abs(Tau));
figure 
plot(t,Tau)

%thermal dynamics
R1=1.748;
R2=1.82;
R=0.844;
a=0.0039;
torque_c=0.231;
title(['Torque over time at ydes=', num2str(ydes)]);
xlabel('time');
ylabel('Torque (Nm)');

initial_temp=25;
winding_temp=initial_temp+(R1+R2)*R*(Tau/torque_c).^2./(1-a*(R1+R2)*R*(Tau/torque_c.^2));
figure 
plot(t, winding_temp)

title(['Motor temp over time at ydes=',num2str(ydes)])
xlabel('time')
ylabel('temp (deg C)')

end

%ODE 
function [dx,Tau]=ODE(r,k,Jm,m,Tau_max,N,g,Kp,Kd,Ki,Kp_s,Kd_s,Ki_s,ydes,yo,t,x)
y=x(1);
y_dot=x(2);
y_int=x(5);

theta=x(3);
theta_dot=x(4);
theta_int=x(6);

y_er=ydes-x(1);
y_ddot=-g+k*(yo-y+r*theta/N)/m;


F_des=Kp*y_er+Kd*(-x(2))+Ki*y_int;
F_des_dot=Kp*(-x(2))+Kd*-y_ddot+Ki*(ydes-x(1));

theta_des=(F_des/k+x(1)-yo)*N/r;
theta_des_dot=(F_des_dot/k+x(2))*N/r;
theta_er=theta_des-x(3);
Tau=Kp_s*theta_er+Kd_s*(theta_des_dot-x(4))+Ki_s*theta_int;

%torque saturation
 if Tau> Tau_max
    Tau=Tau_max;
 end
 if Tau< -Tau_max
    Tau=-Tau_max;
 end


F=k*(yo-x(1)+r/N*x(3));

%state dot
dx(1,:)=x(2);
dx(2,:)=F/m-g;
dx(3,:)=x(4);
dx(4,:)=(Tau-F*r/N)/Jm;
dx(5,:)=ydes-x(1);
dx(6,:)=theta_des-x(3);
end




