function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


g=params.gravity;
m=params.mass;
Ixx=params.Ixx;
Kdz=40;Kpz=260;kdy=40;kpy=260;kpf=100;kdf=50;
% FILL IN YOUR CODE HERE
u1=m*(g+des_state.acc(2)+Kdz*(des_state.vel(2)-state.vel(2))+Kpz*(des_state.pos(2)-state.pos(2)));
if (u1>params.maxF)
    u1=params.maxF;
end

if (u1<params.minF)
    u1=params.minF;
end
fi=(-1/g)*(des_state.acc(1)+kpy*(des_state.pos(1)-state.pos(1))+kdy*(des_state.vel(1)-state.vel(1)));
fid=(-1/g)*(kpy*(des_state.vel(1)-state.vel(1))+kdy*(des_state.acc(1)+  g*state.rot));
u2=Ixx*(0+kpf*(fi-state.rot)+kdf*(fid-state.omega));


