function [F, acc] = position_controller(current_state, desired_state, params, question)


if question ==2
    
    Kp1 = 17;
    Kd1 = 6.6;

    Kp2 = 17;
    Kd2 = 6.6;

    Kp3 = 20;
    Kd3 = 9;
%acc
error_position=current_state.pos-desired_state.pos;
error_velocity=current_state.vel-desired_state.vel;

%Kp=[Kp1; Kp2; Kp3];
%Kp=[8;8;10];
Kp=[30;30;30];
Kd=[Kd1; Kd2; Kd3];
%Kd=[5;5;7];

g=[0; 0 ;params.gravity];
a_desire=desired_state.acc;

acc=a_desire-Kp.*error_position-Kd.*error_velocity;
b3T=[0 0 1];
F=params.mass*b3T*(g+a_desire-Kp.*error_position-Kd.*error_velocity);
       
end



%b3T=[-sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)]; %b3 is actually [0;0;1] b3T is transpose of b3
%%
%question 3
if question ==3
    
      
    Kp1 = 17;
    Kd1 = 6.6;

    Kp2 = 17;
    Kd2 = 6.6;

    Kp3 = 20;
    Kd3 = 9;
    %acc
    error_position=current_state.pos-desired_state.pos;
    error_velocity=current_state.vel-desired_state.vel;

    %Kp=[Kp1; Kp2; Kp3];
    %Kp=[8;8;10];
    Kp=[80;80;95];
    Kd=[Kd1; Kd2; Kd3];

    g=[0; 0 ;params.gravity];
    a_desire=desired_state.acc;

    acc=a_desire-Kp.*error_position-Kd.*error_velocity;
    b3T=[0 0 1];
    F=params.mass*b3T*(g+a_desire-Kp.*error_position-Kd.*error_velocity);
     
end
%%
if question==5
       
    Kp1 = 17;
    Kd1 = 6.6;

    Kp2 = 17;
    Kd2 = 6.6;

    Kp3 = 20;
    Kd3 = 9;
    %acc
    error_position=current_state.pos-desired_state.pos;
    error_velocity=current_state.vel-desired_state.vel;

    %Kp=[Kp1; Kp2; Kp3];
    %Kp=[3;3;5];
    Kp=[30;30;30];
    Kd=[Kd1; Kd2; Kd3];
    %Kd=[4;4;7];
    
    g=[0; 0 ;params.gravity];
    a_desire=desired_state.acc;

    acc=a_desire-Kp.*error_position-Kd.*error_velocity;
    b3T=[0 0 1];
    F=params.mass*b3T*(g+a_desire-Kp.*error_position-Kd.*error_velocity);
     
    
    
    
end

%%
if question ==6
    g=params.gravity;
    psi=current_state.rot(3);
    psid=current_state.omega(3);
    I11=params.inertia(1,1);
    I22=params.inertia(2,2);
    I33=params.inertia(2,2);
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
    F=F(1);
    acc=[0;0;0];
    
    
    
%     A  = [0,0,0,0,0,0,1,0,0,0,0,0;
%           0,0,0,0,0,0,0,1,0,0,0,0;
%           0,0,0,0,0,0,0,0,1,0,0,0;
%           0,0,0,0,0,0,0,0,0,1,0,0;
%           0,0,0,0,0,0,0,0,0,0,1,0;
%           0,0,0,0,0,0,0,0,0,0,0,1;
%           0,0,0,g*sin(psi),g*cos(psi),0,0,0,0,0,0,0;
%           0,0,0,-g*cos(psi),g*sin(psi),0,0,0,0,0,0,0;
%           0,0,0,0,0,0,0,0,0,0,0,0;
%           0,0,0,(J22-J33)*psid^2/J11,0,0,0,0,0,0,(J22-J33)*psid/J11,0;
%           0,0,0,0,(J11-J33)*psid^2/J22,0,0,0,0,-(J11-J33)*psid/J22,0,0;
%           0,0,0,0,0,0,0,0,0,0,0,0];
%     B = [0,0,0,0;
%          0,0,0,0;
%          0,0,0,0;
%          0,0,0,0;
%          0,0,0,0;
%          0,0,0,0;
%          0,0,0,0;
%          0,0,0,0;
%          1/m,0,0,0;
%          0,1/J11,0,0;
%          0,0,1/J22,0;
%          0,0,0,1/J33];
%     Q=0.1*eye(12);
%     R=1*eye(4); 
%     [K,S,P] = lqr(A,B,Q,R);
%     C = [1,0,0,0,0,0,0,0,0,0,0,0;
%           0,1,0,0,0,0,0,0,0,0,0,0;
%           0,0,1,0,0,0,0,0,0,0,0,0;
%           0,0,0,0,0,1,0,0,0,0,0,0];
%     yd = [desired_state.pos;
%            desired_state.rot(3)];
%     x=[
%         current_state.pos ;
%         current_state.rot;
%         current_state.vel ;
%         current_state.omega;
%         ];
%     v = -inv(C*inv(A-B*K)*B)*yd;
%     u = v - K*x + [m*g;0;0;0];
%     
%     F = u(1);
%     acc=[0;0;0];
end


%%

if question==7
       
    Kp1 = 17;
    Kd1 = 6.6;

    Kp2 = 17;
    Kd2 = 6.6;

    Kp3 = 20;
    Kd3 = 9;
    %acc
    error_position=current_state.pos-desired_state.pos;
    error_velocity=current_state.vel-desired_state.vel;

    Kp=[Kp1; Kp2; Kp3];
    %Kp=[5;5;6];
    %Kp=[30;30;30];
    %Kd=[Kd1; Kd2; Kd3];
    Kd=[9;9;14];
    
    
    
    g=[0; 0 ;params.gravity];
    a_desire=desired_state.acc;

    acc=a_desire-Kp.*error_position-Kd.*error_velocity;
    b3T=[0 0 1];
    F=params.mass*b3T*(g+a_desire-Kp.*error_position-Kd.*error_velocity);
     
    
    
    
end

%%

if question==8
       
    Kp1 = 17;
    Kd1 = 6.6;

    Kp2 = 17;
    Kd2 = 6.6;

    Kp3 = 20;
    Kd3 = 9;
    %acc
    error_position=current_state.pos-desired_state.pos;
    error_velocity=current_state.vel-desired_state.vel;

    %Kp=[Kp1; Kp2; Kp3];
    %Kp=[5;5;6];
    Kp=[30;30;30];
    Kd=[Kd1; Kd2; Kd3];
    %Kd=[9;9;14];
    
    
    
    g=[0; 0 ;params.gravity];
    a_desire=desired_state.acc;

    acc=a_desire-Kp.*error_position-Kd.*error_velocity;
    b3T=[0 0 1];
    F=params.mass*b3T*(g+a_desire-Kp.*error_position-Kd.*error_velocity);
     
    
    
    
end

%%

if question==9
       
    Kp1 = 17;
    Kd1 = 6.6;

    Kp2 = 17;
    Kd2 = 6.6;

    Kp3 = 20;
    Kd3 = 9;
    %acc
    error_position=current_state.pos-desired_state.pos;
    error_velocity=current_state.vel-desired_state.vel;

    Kp=[Kp1; Kp2; Kp3];
    %Kp=[5;5;6];
    %Kp=[30;30;30];
    Kd=[Kd1; Kd2; Kd3];
    %Kd=[9;9;14];
    
    
    
    g=[0; 0 ;params.gravity];
    a_desire=desired_state.acc;

    acc=a_desire-Kp.*error_position-Kd.*error_velocity;
    b3T=[0 0 1];
    F=params.mass*b3T*(g+a_desire-Kp.*error_position-Kd.*error_velocity);
     
    
    
    
end

end
