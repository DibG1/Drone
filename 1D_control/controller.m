function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters




% FILL IN YOUR CODE HERE
Kd=20;
Kp=255;
u=params.mass*(0+Kp*(s_des(1)-s(1))+Kd*(s_des(2)-s(2))+params.gravity);

if(u>params.u_max)
    u=params.u_max;

end

if(u<params.u_min)
    u=params.u_min;

end

