function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
rdes=des_state.pos;
vel_des=des_state.vel;
acc_des=des_state.acc;
yaw_des=des_state.yaw;
pdes=0;qdes=0;
r_omega_des=des_state.yawdot;

r=state.pos;
vel=state.vel;
rot=state.rot;
omega=state.omega;

Kp1=[500,500,500];Kd1=[20,20,20];

m=params.mass;
g=params.gravity;
% Thrust
acc_commanded= acc_des+Kp1.*(rdes-r)+Kd1.*(vel_des-vel);
F=params.mass*(g+acc_commanded(3));

% Moment




phi_des=(1/g)*(acc_commanded(1)*sin(yaw_des)-acc_commanded(2)*cos(yaw_des));
theeta_des=(1/g)*(acc_commanded(1)*cos(yaw_des)+acc_commanded(2)*sin(yaw_des));



ep=[phi_des-rot(1);theeta_des-rot(2);yaw_des-rot(3)];
ed=[pdes-omega(1);qdes-omega(2);r_omega_des-omega(3)];

Kp=[100;100;100];Kd=[2;2;2];

M=Kp.*ep+Kd.*ed;


% =================== Your code ends here ===================
if F>params.maxF
    F=params.maxF;
end
if F<params.minF
        F=params.minF;
end
    
          

