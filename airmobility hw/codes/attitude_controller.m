function [M] = attitude_controller(current_state,desired_state,params,question)
%% for question 2
    
    if question==2
       
        % Example PD gains
        Kpphi = 190;
        Kdphi = 30;

        Kptheta = 198;
        Kdtheta = 30;

        Kppsi = 80;
        Kdpsi = 17.88;
        
        %Kr=[Kpphi;Kptheta;Kppsi];
        %Kr=[130;130;60];
        Kr=[380;380;160];
        
        Kw=[Kdphi;Kdtheta;Kdpsi];
        %Kw=[45;45;32];
        

        %rotational matrix:
        R=r_func(current_state);
        Rd=r_func(desired_state);

        %error setups
        error_r=current_state.rot-desired_state.rot;
        error_w=current_state.omega-R.'*Rd*desired_state.omega;
        %error_r=current_state.rot-desired_state.rot;
        %error_w=current_state.omega-desired_state.omega;
        
        %output
        M=params.inertia*(-Kr.*error_r-Kw.*error_w); %u2
    
        
    end
    
    %% for question 3
     if question==3
        % Example PD gains
        Kpphi = 190;
        Kdphi = 30;

        Kptheta = 198;
        Kdtheta = 30;

        Kppsi = 80;
        Kdpsi = 17.88;
        
        Kr=[Kpphi;Kptheta;Kppsi];
        %Kr=[100;100;40];
        %Kr=[400;400;160];
        
        Kw=[Kdphi;Kdtheta;Kdpsi];

        %rotational matrix:
        R=r_func(current_state);
        Rd=r_func(desired_state);

        %error setups
        error_r=current_state.rot-desired_state.rot;
        error_w=current_state.omega-R.'*Rd*desired_state.omega;
        %error_r=current_state.rot-desired_state.rot;
        %error_w=current_state.omega-desired_state.omega;
        
        %output
        M=params.inertia*(-Kr.*error_r-Kw.*error_w); %u2
     end
     
     %%
     if question ==5
    % Example PD gains
        Kpphi = 190;
        Kdphi = 30;

        Kptheta = 198;
        Kdtheta = 30;

        Kppsi = 80;
        Kdpsi = 17.88;
        
        
        Kr=[Kpphi;Kptheta;Kppsi];
        %Kr=[100;100;40];
        %Kr=[400;400;160];
        
        Kw=[Kdphi;Kdtheta;Kdpsi];
        %Kw=[25;25;14];
        
        %rotational matrix:
        R=r_func(current_state);
        Rd=r_func(desired_state);

        %error setups
        error_r=current_state.rot-desired_state.rot;
        error_w=current_state.omega-R.'*Rd*desired_state.omega;
        %error_r=current_state.rot-desired_state.rot;
        %error_w=current_state.omega-desired_state.omega;
        
        %output
        M=params.inertia*(-Kr.*error_r-Kw.*error_w); %u2

    
     end
     
     %%
    if question==6
    g=params.gravity;
    psi=current_state.rot(3);
    psid=current_state.omega(3);
    I11=params.inertia(1,1);
    I22=params.inertia(2,2);
    I33=params.inertia(3,3);
    m=params.mass;
    
    
    
    A=[0 0 0 0                        0                        0 1 0 0 0                   0                  0 ;
       0 0 0 0                        0                        0 0 1 0 0                   0                  0 ;
       0 0 0 0                        0                        0 0 0 1 0                   0                  0 ;
       0 0 0 0                        0                        0 0 0 0 1                   0                  0 ;
       0 0 0 0                        0                        0 0 0 0 0                   1                  0 ;
       0 0 0 0                        0                        0 0 0 0 0                   0                  1 ;
       0 0 0 g*sin(psi)               g*cos(psi)               0 0 0 0 0                   0                  0 ;
       0 0 0 -g*cos(psi)              g*sin(psi)               0 0 0 0 0                   0                  0 ;
       0 0 0 0                        0                        0 0 0 0 0                   0                  0 ;
       0 0 0 (I22-I33)*((psid)^2)/I11 0                        0 0 0 0 0                   (I22-I33)*psid/I11 0 ;
       0 0 0 0                        (I11-I33)*((psid)^2)/I22 0 0 0 0 -(I11-I33)*psid/I22 0                  0 ;
       0 0 0 0                        0                        0 0 0 0 0                   0                  0 ;
      ];
    B=[
        0   0     0     0 ;
        0   0     0     0 ;
        0   0     0     0 ;
        0   0     0     0 ;
        0   0     0     0 ;
        0   0     0     0 ;
        0   0     0     0 ;
        0   0     0     0 ;
        1/m 0     0     0 ;
        0   1/I11 0     0 ;
        0   0     1/I22 0;
        0   0     0     1/I33;
    ];
  
    C=[
        1 0 0 0 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0 0 0 0;
        0 0 1 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0 0 0 0 0; 
      ];

        
    % q=0.1 R=2 
    Q=0.1*eye(12);
    R=0.5*eye(4);
    [K,S,P]=lqr(A,B,Q,R);

    yd=[desired_state.pos; desired_state.rot(3)];
    v=-inv(C*inv(A-B*K)*B)*yd;
    
    x=[
        current_state.pos ;
        current_state.rot;
        current_state.vel ;
        current_state.omega;
        ];
    
    F=v-K*x+[m*g;0;0;0];
    M=F(2:4);

    
    
    
    end 
     
     %%
    if question ==7
    % Example PD gains
        Kpphi = 190;
        Kdphi = 30;

        Kptheta = 198;
        Kdtheta = 30;

        Kppsi = 80;
        Kdpsi = 17.88;
        
        Kr=[Kpphi;Kptheta;Kppsi];
        %Kr=[100;100;40];
        %Kr=[400;400;160];
        
        Kw=[Kdphi;Kdtheta;Kdpsi];

        %rotational matrix:
        R=r_func(current_state);
        Rd=r_func(desired_state);

        %error setups
        error_r=current_state.rot-desired_state.rot;
        error_w=current_state.omega-R.'*Rd*desired_state.omega;
        %error_r=current_state.rot-desired_state.rot;
        %error_w=current_state.omega-desired_state.omega;
        
        %output
        M=params.inertia*(-Kr.*error_r-Kw.*error_w); %u2

    
    end
    %%
    if question ==8
    % Example PD gains
        Kpphi = 190;
        Kdphi = 30;

        Kptheta = 198;
        Kdtheta = 30;

        Kppsi = 80;
        Kdpsi = 17.88;
        
        %Kr=[Kpphi;Kptheta;Kppsi];
        %Kr=[100;100;40];
        Kr=[400;400;160];
        
        Kw=[Kdphi;Kdtheta;Kdpsi];

        %rotational matrix:
        R=r_func(current_state);
        Rd=r_func(desired_state);

        %error setups
        error_r=current_state.rot-desired_state.rot;
        error_w=current_state.omega-R.'*Rd*desired_state.omega;
        %error_r=current_state.rot-desired_state.rot;
        %error_w=current_state.omega-desired_state.omega;
        
        %output
        M=params.inertia*(-Kr.*error_r-Kw.*error_w); %u2

    
    end
    
     %%
    if question ==9
    % Example PD gains
        Kpphi = 190;
        Kdphi = 30;

        Kptheta = 198;
        Kdtheta = 30;

        Kppsi = 80;
        Kdpsi = 17.88;
        
        Kr=[Kpphi;Kptheta;Kppsi];
        %Kr=[100;100;40];
        %Kr=[400;400;160];
        
        Kw=[Kdphi;Kdtheta;Kdpsi];

        %rotational matrix:
        R=r_func(current_state);
        Rd=r_func(desired_state);

        %error setups
        error_r=current_state.rot-desired_state.rot;
        error_w=current_state.omega-R.'*Rd*desired_state.omega;
        %error_w=current_state.omega-desired_state.omega;
        
        %output
        M=params.inertia*(-Kr.*error_r-Kw.*error_w); %u2

    
    end
    

end




function R=r_func(state)
 phi=state.rot(1);
 theta=state.rot(2);
 psi=state.rot(3);
 Rx=[1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
 Ry=[cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
 Rz=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
 R=Rz*Ry*Rx;
end


