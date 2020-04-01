function [] = plot_quadrotor_errors(state, state_des,time_vector)

%% Get individual plots from state

pos = state(1:3,:);
vel = state(4:6,:);
rpy = state(7:9,:);
ang_vel = state(10:12,:);
acc = state(13:15,:);

pos_des = state_des(1:3,:);
vel_des = state_des(4:6,:);
rpy_des = state_des(7:9,:);
ang_vel_des = state_des(10:12,:);
acc_des = state_des(13:15,:);

%% Get error from desired and actual

error_pos = pos-pos_des;
error_vel = vel-vel_des;
error_rpy = rpy-rpy_des;
error_ang_vel = ang_vel-ang_vel_des;
error_acc = acc - acc_des;
% No error in rpm, assume perfect motor speed control

%% Plot error

% Plot position error

labels = {'x [m]', 'y [m]', 'z [m]'};
title_name = {'Error in x','Error in y','Error in z'};
str = 'Plot of Error';
%figure(1)
figure('Name',str);

for i = 1:3
    subplot(5, 3, i)
    plot(time_vector,error_pos(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end




% Plot orientation error

labels = {'\phi', '\theta', '\psi'};
title_name = {'Error in \phi','Error in \theta','Error in \psi'};

for i = 1:3
    subplot(5, 3, i+3)
    plot(time_vector,error_rpy(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end

% Plot velocity error

labels = {'vx [m/s]', 'vy [m/s]', 'vz [m/s]'};
title_name = {'Error in v_x','Error in v_y','Error in v_z'};

for i = 1:3
    subplot(5, 3, i+6)
    plot(time_vector,error_vel(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end

% Plot angular velocity error

labels = {'\omega x [rad/s]', '\omega y [rad/s]', '\omega z [rad/s]'};
title_name = {'Error in \omega_x','Error in \omega_y','Error in \omega_z'};

for i = 1:3
    subplot(5, 3, i+9)
    plot(time_vector,error_ang_vel(i,:),'r');
    
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end

labels = {'ax [m/s^2]', 'ay [m/s^2]', 'az [m/s^2]'};
title_name = {'Error in a_x','Error in a_y','Error in a_z'};

for i = 1:3
    subplot(5, 3, i+12)
    plot(time_vector,error_acc(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end
%% plot cumulative error

% Plot cumulative position error

labels = {'x [m]', 'y [m]', 'z [m]'};
title_name = {'Cumulative Error in x','Cumulative Error in y','Cumulative Error in z'};
str = 'Plot of Cumulative Error';
figure('Name',str);

error_pos_cum=zeros(3,length(error_pos(1,:)));
error_pos_cum(1,1)=error_pos(1,1);
error_pos_cum(2,1)=error_pos(2,1);
error_pos_cum(3,1)=error_pos(3,1);


for i=1:3
    for j=2:length(error_pos(i,:))
        error_pos_cum(i,j)=error_pos_cum(i,j-1)+error_pos(i,j);
    end
end    


for i = 1:3
    subplot(5, 3, i)
    
    plot(time_vector,error_pos_cum(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end




% Plot cumulative orientation error

labels = {'\phi', '\theta', '\psi'};
title_name = {'Cumulative Error in \phi','Cumulative Error in \theta','Cumulative Error in \psi'};

error_rpy_cum=zeros(3,length(error_rpy(1,:)));
error_rpy_cum(1,1)=error_rpy(1,1);
error_rpy_cum(2,1)=error_rpy(2,1);
error_rpy_cum(3,1)=error_rpy(3,1);


for i=1:3
    for j=2:length(error_rpy(i,:))
        error_rpy_cum(i,j)=error_rpy_cum(i,j-1)+error_rpy(i,j);
    end
end    

for i = 1:3
    subplot(5, 3, i+3)
    plot(time_vector,error_rpy_cum(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end

% Plot cumulative velocity error

labels = {'vx [m/s]', 'vy [m/s]', 'vz [m/s]'};
title_name = {'Cumulative Error in v_x','Cumulative Error in v_y','Cumulative Error in v_z'};

error_vel_cum=zeros(3,length(error_vel(1,:)));
error_vel_cum(1,1)=error_vel(1,1);
error_vel_cum(2,1)=error_vel(2,1);
error_vel_cum(3,1)=error_vel(3,1);


for i=1:3
    for j=2:length(error_vel(i,:))
        error_vel_cum(i,j)=error_vel_cum(i,j-1)+error_vel(i,j);
    end
end    


for i = 1:3
    subplot(5, 3, i+6)
    plot(time_vector,error_vel_cum(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end

% Plot cumulative angular velocity error

labels = {'\omega x [rad/s]', '\omega y [rad/s]', '\omega z [rad/s]'};
title_name = {'Cumulative Error in \omega_x','Cumulative Error in \omega_y','Cumulative Error in \omega_z'};

error_angvel_cum=zeros(3,length(error_ang_vel(1,:)));
error_angvel_cum(1,1)=error_ang_vel(1,1);
error_angvel_cum(2,1)=error_ang_vel(2,1);
error_angvel_cum(3,1)=error_ang_vel(3,1);


for i=1:3
    for j=2:length(error_ang_vel(i,:))
        error_angvel_cum(i,j)=error_angvel_cum(i,j-1)+error_ang_vel(i,j);
    end
end    


for i = 1:3
    subplot(5, 3, i+9)
    plot(time_vector,error_angvel_cum(i,:),'r');
    
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end

labels = {'ax [m/s^2]', 'ay [m/s^2]', 'az [m/s^2]'};
title_name = {'Cumulative Error in a_x','Cumulative Error in a_y','Cumulative Error in a_z'};

error_acc_cum=zeros(3,length(error_acc(1,:)));
error_acc_cum(1,1)=error_acc(1,1);
error_acc_cum(2,1)=error_acc(2,1);
error_acc_cum(3,1)=error_acc(3,1);


for i=1:3
    for j=2:length(error_acc(i,:))
        error_acc_cum(i,j)=error_acc_cum(i,j-1)+error_acc(i,j);
    end
end    


for i = 1:3
    subplot(5, 3, i+12)
    plot(time_vector,error_acc_cum(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end




%% Plot desired and actual


% Plot Position
labels = {'x [m]', 'y [m]', 'z [m]'};
title_name = {'Position in x','Position in y','Position in z'};

str = 'Position';
figure('Name',str);
%figure (2)
%pos(3,:)=pos(3,:)+0.05;
for i = 1:3
    subplot(5, 3, i)
    plot(time_vector, pos_des(i,:),'b',time_vector, pos(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
    legend('Desired','Actual');
end

%passing variables to workspace for verification
assignin('base','z',pos(3,:));
assignin('base','time',time_vector);

%percent overshoot
zmax=max(pos(3,:));
percent_overshoot=(zmax-0.125)/0.125  %percent overshoot
%steady state value
steady_state_value=pos(3,end)
%rise time
idx1=find(abs(pos(3,:)-0.9*0.125)<0.0005,1)%index for rise time
rise_time=time_vector(idx1) %startcounting after hover  
%settling time
idx=find(abs(pos(3,:)-0.125)>0.01,1,'last') %index for settling time
settling_time=time_vector(idx)

% Plot Attitude
labels = {'\phi', '\theta', '\psi'};
title_name = {'Orientation in \phi','Orientation in \theta','Orientation in \psi'};

for i = 1:3
    subplot(5, 3, i+3)
    plot(time_vector, rpy_des(i,:), 'b', time_vector, rpy(i,:), 'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
    legend('Desired','Actual');
end



% Plot velocity

labels = {'vx [m/s]', 'vy [m/s]', 'vz [m/s]'};
title_name = {'Velocity in x','Velocity in y','Velocity in z'};

for i = 1:3
    subplot(5, 3, i+6)
    plot(time_vector, vel_des(i,:), 'b', time_vector, vel(i,:), 'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
end


% Plot angular velocity
labels = {'\omega x [rad/s]', '\omega y [rad/s]', '\omega z [rad/s]'};
title_name = {'Angular Velocity in x','Angular Velocity in y','Angular Velocity in z'};


for i = 1:3
    subplot(5, 3, i+9)
    plot(time_vector, ang_vel_des(i,:),'b',time_vector, ang_vel(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
    legend('Desired','Actual');
end

% Plot Acceleration
labels = {'ax [m/s^2]', 'ay [m/s^2]', 'az [m/s^2]'};
title_name = {'Acceleration in x','Acceleration in y','Acceleration in z'};

for i = 1:3
    subplot(5, 3, i+12)
    plot(time_vector,acc_des(i,:),'b',time_vector,acc(i,:),'r');
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
    title(title_name{i})
    legend('Desired','Actual');
end
 %print max acceleration
    Max_acc=max(acc(3,:))
    min_acc=min(acc(3,:))
end

