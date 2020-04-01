function [waypoints, waypoint_times] = lookup_waypoints(question)
%%
if question == 2
    waypoints = [0 0.1 0.2 0.3; 0 0 0 0 ;  0.5 0.5 0.5 0.5; 0 0 0 0]; % first row: x, second row: y, third row: z, forth row: yaw
    waypoint_times = [0 2 4 6];

end

%%
if question==3
    
    way_x=linspace(0, 0,210); %x
    way_y=linspace(0, 0, 210);  %y
    way_z1=linspace(0.05,1, 90);   %z
    way_z2=linspace(1,1, 30);
    way_z3=linspace(1,0.05,90);
    way_psi=linspace(0,0,210);  %psi
    way_zdot1=linspace(0.25,0.25,89);  %zdot
    way_zdot2=linspace(0,0,30);  %zdot
    way_zdot3=linspace(-0.25,-0.25,89);  %zdot
    
    waypoints=[ way_x;
                way_y; 
                way_z1 way_z2 way_z3;
                way_psi;
                0 way_zdot1 way_zdot2 way_zdot3 0;
                ];
    waypoint_times=linspace(0,10,210);
end

%%
if question ==5
    
    timespan_track=16;
    way_x=linspace(0, 0,160);           %x position
    way_y=linspace(0, 0, 160);          %y position
    
    way_z1=linspace(0,0.025, 20);       %takeoff state
    way_z2=linspace(0.025,0.025, 50);   %hover state
    way_z3=linspace(0.125,0.125,20);    %tracking state
    way_z4=linspace(0.125,0.125,50);    %hover state
    way_z5=linspace(0.125,0.125,20);            %landing state
    
    ang=0*pi()/180; % 0 for part 1, and 15 for part 2
    
    way_psi1=linspace(0,0,70);          
    way_psi2=linspace(ang,ang,90);      %psi
    
    waypoints=[ way_x;
                way_y; 
                way_z1 way_z2 way_z3 way_z4 way_z5;
                way_psi1 way_psi2;
                ];
    waypoint_times=linspace(0,timespan_track,160);
end

%%
if question == 6
    %q2
    %waypoints = [0 0.1 0.2 0.3; 0 0 0 0 ;  0.5 0.5 0.5 0.5; 0 0 0 0]; % first row: x, second row: y, third row: z, forth row: yaw
    %waypoint_times = [0 2 4 6];
    
    %q3
%     way_x=linspace(0, 0,210); %x
%     way_y=linspace(0, 0, 210);  %y
%     way_z1=linspace(0.05,1, 90);   %z
%     way_z2=linspace(1,1, 30);
%     way_z3=linspace(1,0.05,90);
%     way_psi=linspace(0,0,210);  %psi
%     way_zdot1=linspace(0.25,0.25,89);  %zdot
%     way_zdot2=linspace(0,0,30);  %zdot
%     way_zdot3=linspace(-0.25,-0.25,89);  %zdot
%     
%     waypoints=[ way_x;
%                 way_y; 
%                 way_z1 way_z2 way_z3;
%                 way_psi;
%                 0 way_zdot1 way_zdot2 way_zdot3 0;
%                 ];
%     waypoint_times=linspace(0,10,210);
    
    %q5
    timespan_track=16;
    way_x=linspace(0, 0,160);           %x position
    way_y=linspace(0, 0, 160);          %y position
    
    way_z1=linspace(0,0.025, 20);       %takeoff state
    way_z2=linspace(0.025,0.025, 50);   %hover state
    way_z3=linspace(0.125,0.125,20);    %tracking state
    way_z4=linspace(0.125,0.125,50);    %hover state
    way_z5=linspace(0,0,20);            %landing state
    
    ang=15*pi()/180;
    
    way_psi1=linspace(0,0,70);          
    way_psi2=linspace(ang,ang,90);      %psi
    
    waypoints=[ way_x;
                way_y; 
                way_z1 way_z2 way_z3 way_z4 way_z5;
                way_psi1 way_psi2;
                ];
    waypoint_times=linspace(0,timespan_track,160);    
    
    
    
    
end


%%
if question ==7
    
    %takeoff and hover
    waypoint_times_pre=linspace(0,8,80); 
    waypoints_x_pre=linspace(0,0,80);
    waypoints_y_pre=linspace(0,0,80);
    waypoints_z1=linspace(0,1, 30);   
    waypoints_z2=linspace(1,1, 50);
    waypoints_psi_pre=linspace(0,0,80);   
  
         
    %track
    timespan_track=0.1;
    waypoint_times_track=linspace(8,8+timespan_track,1000);
    for idx=1:length(waypoint_times_track)
            waypoints_x(idx)=0;
            waypoints_y(idx)=0;
            waypoints_z3(idx)=-(9/(timespan_track^2)).*(waypoint_times_track(idx)-timespan_track-8).^2+10; %polynomial
            %waypoints_z3(idx)=(9/timespan_track)*(waypoint_times_track(idx)-8)+1; %line
            waypoints_psi(idx)=0;    
    end
   
    
    %hover for 5s
    waypoint_times_hover=linspace(8+timespan_track,8+timespan_track+5,10);
    waypoints_x_hover=linspace(waypoints_x(end),waypoints_x(end),10);
    waypoints_y_hover=linspace(waypoints_y(end),waypoints_y(end),10);
    waypoints_z_hover=linspace(waypoints_z3(end),waypoints_z3(end),10);
    waypoints_psi_hover=linspace(0,0,10);
   
    
    %land
    waypoint_times_landing=linspace(8+timespan_track+5,8+timespan_track+5+80,100);
    waypoints_x_landing=linspace(waypoints_x_hover(end),waypoints_x_hover(end),100);
    waypoints_y_landing=linspace(waypoints_y_hover(end),waypoints_y_hover(end),100);
    waypoints_z_landing=linspace(waypoints_z_hover(end),0,100);
    waypoints_psi_landing=linspace(0,0,100);
    %waypoints_xdot_landing=linspace(0,0,10);
    %waypoints_ydot_landing=linspace(0,0,10);
    
    
    %%overall waypoint_times and waypoints
    waypoints=[waypoints_x_pre waypoints_x waypoints_x_hover waypoints_x_landing;
               waypoints_y_pre waypoints_y waypoints_y_hover waypoints_y_landing;
               waypoints_z1 waypoints_z2 waypoints_z3 waypoints_z_hover waypoints_z_landing;
               waypoints_psi_pre waypoints_psi waypoints_psi_hover waypoints_psi_landing]; 
    waypoint_times=[waypoint_times_pre waypoint_times_track waypoint_times_hover waypoint_times_landing ];
    
end


%%
%question 8
if question ==8
   
    %part1
    %{
    %take off
    %hover
    timespan_takeoff=8;
    waypoint_times_pre=linspace(0,timespan_takeoff,80); 
    waypoints_x_pre=linspace(0,0,80);
    waypoints_y_pre=linspace(0,0,80);
    waypoints_z_pre=linspace(0,1, 30);   
    waypoints_z_hover1=linspace(1,1, 50);
    waypoints_psi_pre=linspace(0,0,80); 
    waypoints_xdot_pre=linspace(0,0,80);
    waypoints_ydot_pre=linspace(0,0,80);
  
        
    %track  
    loop=1;
    timespan_track=loop*4*pi;
    waypoint_times_track=linspace(timespan_takeoff,timespan_takeoff+timespan_track,100);
    track_ang=linspace(-pi/2,(4*loop-1)*pi/2,100);
    ang_dot=0.5;

    for idx=1:length(waypoint_times_track)
            waypoints_x(idx)=2*cos(track_ang(idx));
            waypoints_y(idx)=1+sin(track_ang(idx));
            waypoints_z(idx)=1;
            waypoints_psi(idx)=0;
            waypoints_ydot(idx)=0;
           
    end
   
    waypoints_xdot1=linspace(0,0,90);
    waypoints_xdot2=linspace(0,1,10);
    
    
    %slow down and stop
    waypoint_times_stop=linspace(timespan_takeoff+timespan_track,timespan_takeoff+timespan_track+2,10);
    waypoints_x_stop=linspace(waypoints_x(end),waypoints_x(end)+2,10);
    waypoints_y_stop=linspace(waypoints_y(end),waypoints_y(end),10);
    waypoints_z_stop=linspace(waypoints_z(end),waypoints_z(end),10);
    waypoints_psi_stop=linspace(0,0,10);
    waypoints_xdot_stop=linspace(waypoints_xdot2(end),0,10);
    waypoints_ydot_stop=linspace(0,0,10);
    
    %hover for 5s
    waypoint_times_hover=linspace(timespan_takeoff+timespan_track+2,timespan_takeoff+timespan_track+7,10);
    waypoints_x_hover=linspace(waypoints_x_stop(end),waypoints_x_stop(end),10);
    waypoints_y_hover=linspace(waypoints_y_stop(end),waypoints_y_stop(end),10);
    waypoints_z_hover2=linspace(waypoints_z_stop(end),waypoints_z_stop(end),10);
    waypoints_psi_hover=linspace(0,0,10);
    waypoints_xdot_hover=linspace(0,0,10);
    waypoints_ydot_hover=linspace(0,0,10);
    
    %land
    waypoint_times_landing=linspace(timespan_takeoff+timespan_track+7,timespan_takeoff+timespan_track+10,80);
    waypoints_x_landing=linspace(waypoints_x_hover(end),waypoints_x_hover(end),80);
    waypoints_y_landing=linspace(waypoints_y_hover(end),waypoints_y_hover(end),80);
    waypoints_z_landing=linspace(waypoints_z_hover2(end),0,80);
    waypoints_psi_landing=linspace(0,0,80);
    waypoints_xdot_landing=linspace(0,0,80);
    waypoints_ydot_landing=linspace(0,0,80);
    
    
    %overall waypoint_times and waypoints
    waypoint_times=[waypoint_times_pre waypoint_times_track waypoint_times_stop waypoint_times_hover waypoint_times_landing];
    waypoints=[waypoints_x_pre waypoints_x waypoints_x_stop waypoints_x_hover waypoints_x_landing;
               waypoints_y_pre waypoints_y waypoints_y_stop waypoints_y_hover waypoints_y_landing;
               waypoints_z_pre waypoints_z_hover1 waypoints_z waypoints_z_stop waypoints_z_hover2 waypoints_z_landing;
               waypoints_psi_pre waypoints_psi waypoints_psi_stop waypoints_psi_hover waypoints_psi_landing;
               waypoints_xdot_pre waypoints_xdot1 waypoints_xdot2 waypoints_xdot_stop waypoints_xdot_hover waypoints_xdot_landing;
               waypoints_ydot_pre waypoints_ydot waypoints_ydot_stop waypoints_ydot_hover waypoints_ydot_landing]; 
    
    %}
    
    
    %part2
    %take off
    %hover
    timespan_takeoff=8;
    waypoint_times_pre=linspace(0,timespan_takeoff,80); 
    waypoints_x_pre=linspace(0,0,80);
    waypoints_y_pre=linspace(0,0,80);
    waypoints_z_pre=linspace(0,1, 30);   
    waypoints_z_hover1=linspace(1,1, 50);
    waypoints_psi_pre=linspace(0,0,80); 
    waypoints_xdot_pre=linspace(0,0,80);
    waypoints_ydot_pre=linspace(0,0,80);
  
        
    %track  
    loop=2;
    timespan_track=loop*4*pi;
    waypoint_times_track=linspace(timespan_takeoff,timespan_takeoff+timespan_track,100);
    track_ang=linspace(-pi/2,(4*loop-1)*pi/2,100);
    %speed=1; %tracking velocity
    speed=0.5;

    for idx=1:length(waypoint_times_track)
            ang_dot=speed/sqrt(1+3*sin(track_ang(idx))^2);
            waypoints_x(idx)=2*cos(track_ang(idx));
            waypoints_y(idx)=1+sin(track_ang(idx));
            waypoints_z(idx)=1;
            waypoints_psi(idx)=0;
            waypoints_xdot(idx)=-2*sin(track_ang(idx))*ang_dot;
            waypoints_ydot(idx)=cos(track_ang(idx))*ang_dot;
    end
   
    %slow down and stop
    waypoint_times_stop=linspace(timespan_takeoff+timespan_track,timespan_takeoff+timespan_track+2,10);
    waypoints_x_stop=linspace(waypoints_x(end),waypoints_x(end)+2,10);
    waypoints_y_stop=linspace(waypoints_y(end),waypoints_y(end),10);
    waypoints_z_stop=linspace(waypoints_z(end),waypoints_z(end),10);
    waypoints_psi_stop=linspace(0,0,10);
    waypoints_xdot_stop=linspace(waypoints_xdot(end),0,10);
    waypoints_ydot_stop=linspace(0,0,10);
    
    %hover for 5s
    waypoint_times_hover=linspace(timespan_takeoff+timespan_track+2,timespan_takeoff+timespan_track+7,10);
    waypoints_x_hover=linspace(waypoints_x_stop(end),waypoints_x_stop(end),10);
    waypoints_y_hover=linspace(waypoints_y_stop(end),waypoints_y_stop(end),10);
    waypoints_z_hover2=linspace(waypoints_z_stop(end),waypoints_z_stop(end),10);
    waypoints_psi_hover=linspace(0,0,10);
    waypoints_xdot_hover=linspace(0,0,10);
    waypoints_ydot_hover=linspace(0,0,10);
    
    %land
    waypoint_times_landing=linspace(timespan_takeoff+timespan_track+7,timespan_takeoff+timespan_track+10,10);
    waypoints_x_landing=linspace(waypoints_x_hover(end),waypoints_x_hover(end),10);
    waypoints_y_landing=linspace(waypoints_y_hover(end),waypoints_y_hover(end),10);
    waypoints_z_landing=linspace(waypoints_z_hover2(end),0,10);
    waypoints_psi_landing=linspace(0,0,10);
    waypoints_xdot_landing=linspace(0,0,10);
    waypoints_ydot_landing=linspace(0,0,10);
    
    
    %overall waypoint_times and waypoints
    waypoint_times=[waypoint_times_pre waypoint_times_track waypoint_times_stop waypoint_times_hover waypoint_times_landing];
    waypoints=[waypoints_x_pre waypoints_x waypoints_x_stop waypoints_x_hover waypoints_x_landing;
               waypoints_y_pre waypoints_y waypoints_y_stop waypoints_y_hover waypoints_y_landing;
               waypoints_z_pre waypoints_z_hover1 waypoints_z waypoints_z_stop waypoints_z_hover2 waypoints_z_landing;
               waypoints_psi_pre waypoints_psi waypoints_psi_stop waypoints_psi_hover waypoints_psi_landing;
               waypoints_xdot_pre waypoints_xdot waypoints_xdot_stop waypoints_xdot_hover waypoints_xdot_landing;
               waypoints_ydot_pre waypoints_ydot waypoints_ydot_stop waypoints_ydot_hover waypoints_ydot_landing]; 
    
    
end

%%
%question 9
if question ==9
   
    %part2
    %take off
    %hover
    timespan_takeoff=8;
    waypoint_times_pre=linspace(0,timespan_takeoff,80); 
    waypoints_x_pre=linspace(0,0,80);
    waypoints_y_pre=linspace(0,0,80);
    waypoints_z_pre=linspace(0,1, 30);   
    waypoints_z_hover1=linspace(1,1, 50);
    waypoints_psi_pre=linspace(0,0,80); 
    waypoints_xdot_pre=linspace(0,0,80);
    waypoints_ydot_pre=linspace(0,0,80);
  
        
    %track  
    loop=2;
    timespan_track=loop*4*pi;
    waypoint_times_track=linspace(timespan_takeoff,timespan_takeoff+timespan_track,100);
    track_ang=linspace(-pi/2,(4*loop-1)*pi/2,100);
    

    for idx=1:length(waypoint_times_track)
            ang_dot=1/sqrt(1+3*sin(track_ang(idx))^2);
            waypoints_x(idx)=2*cos(track_ang(idx));
            waypoints_y(idx)=1+sin(track_ang(idx));
            waypoints_z(idx)=1;
            waypoints_psi(idx)=track_ang(idx)+pi;
            waypoints_xdot(idx)=-2*sin(track_ang(idx))*ang_dot;
            waypoints_ydot(idx)=cos(track_ang(idx))*ang_dot;
    end
   
    %slow down and stop
    waypoint_times_stop=linspace(timespan_takeoff+timespan_track,timespan_takeoff+timespan_track+2,10);
    waypoints_x_stop=linspace(waypoints_x(end),waypoints_x(end)+2,10);
    waypoints_y_stop=linspace(waypoints_y(end),waypoints_y(end),10);
    waypoints_z_stop=linspace(waypoints_z(end),waypoints_z(end),10);
    waypoints_psi_stop=linspace(waypoints_psi(end),waypoints_psi(end),10);
    waypoints_xdot_stop=linspace(waypoints_xdot(end),0,10);
    waypoints_ydot_stop=linspace(0,0,10);
    
    %hover for 5s
    waypoint_times_hover=linspace(timespan_takeoff+timespan_track+2,timespan_takeoff+timespan_track+7,10);
    waypoints_x_hover=linspace(waypoints_x_stop(end),waypoints_x_stop(end),10);
    waypoints_y_hover=linspace(waypoints_y_stop(end),waypoints_y_stop(end),10);
    waypoints_z_hover2=linspace(waypoints_z_stop(end),waypoints_z_stop(end),10);
    waypoints_psi_hover=linspace(waypoints_psi_stop(end),waypoints_psi_stop(end),10);
    waypoints_xdot_hover=linspace(0,0,10);
    waypoints_ydot_hover=linspace(0,0,10);
    
    %land
    waypoint_times_landing=linspace(timespan_takeoff+timespan_track+7,timespan_takeoff+timespan_track+10,10);
    waypoints_x_landing=linspace(waypoints_x_hover(end),waypoints_x_hover(end),10);
    waypoints_y_landing=linspace(waypoints_y_hover(end),waypoints_y_hover(end),10);
    waypoints_z_landing=linspace(waypoints_z_hover2(end),0,10);
    waypoints_psi_landing=linspace(waypoints_psi_stop(end),waypoints_psi_stop(end),10);
    waypoints_xdot_landing=linspace(0,0,10);
    waypoints_ydot_landing=linspace(0,0,10);
    
    
    %overall waypoint_times and waypoints
    waypoint_times=[waypoint_times_pre waypoint_times_track waypoint_times_stop waypoint_times_hover waypoint_times_landing];
    waypoints=[waypoints_x_pre waypoints_x waypoints_x_stop waypoints_x_hover waypoints_x_landing;
               waypoints_y_pre waypoints_y waypoints_y_stop waypoints_y_hover waypoints_y_landing;
               waypoints_z_pre waypoints_z_hover1 waypoints_z waypoints_z_stop waypoints_z_hover2 waypoints_z_landing;
               waypoints_psi_pre waypoints_psi waypoints_psi_stop waypoints_psi_hover waypoints_psi_landing;
               waypoints_xdot_pre waypoints_xdot waypoints_xdot_stop waypoints_xdot_hover waypoints_xdot_landing;
               waypoints_ydot_pre waypoints_ydot waypoints_ydot_stop waypoints_ydot_hover waypoints_ydot_landing]; 
    
    
end

end


    