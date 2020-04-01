close all
clear all
m=80;
r=0.05;
k=20000;
Jm=0.000506;
Tau_max=1.313;
N=40;
g=9.81;
yo=1;

Kp=16000;
Kd=100;
Ki=7000;
Kp_s=2;
Kd_s=2;
Ki_s=0.5;

for ydes=0.7:0.1:0.9

sim('p2_4',100)

figure
plot(tout,y)
title(['y position over time at ydes=', num2str(ydes)]);
xlabel('time');
ylabel('y (m)');

figure
plot(tout,tau)
title(['Torque over time at ydes=', num2str(ydes)]);
xlabel('time');
ylabel('Torque (Nm)');

%thermal dynamics
R1=1.748;
R2=1.82;
R=0.844;
a=0.0039;
torque_c=0.231;

initial_temp=25;
winding_temp=initial_temp+(R1+R2)*R*(tau/torque_c).^2./(1-a*(R1+R2)*R*(tau/torque_c.^2));
figure 
plot(tout, winding_temp)

title(['Motor temp over time at ydes=',num2str(ydes)])
xlabel('time')
ylabel('temp (deg C)')
end



