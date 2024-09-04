%  ftrackgenerator - in Cartesian coordinates
function [xk,vk]=ftrackgenerator(xk_1,vk_1,ak_1,T,q)
 
q1=q(1,1);
q2=q(2,1);
q3=q(3,1);
xk(1,1)=xk_1(1,1)+vk_1(1,1)*T+0.5*ak_1(1,1)*T^2+0.5*normrnd(0,q1)*T^2;
xk(2,1)=xk_1(2,1)+vk_1(2,1)*T+0.5*ak_1(2,1)*T^2+0.5*normrnd(0,q2)*T^2;
xk(3,1)=xk_1(3,1)+vk_1(3,1)*T+0.5*ak_1(3,1)*T^2+0.5*normrnd(0,q3)*T^2;
vk(1,1)=vk_1(1,1)+ak_1(1,1)*T;
vk(2,1)=vk_1(2,1)+ak_1(2,1)*T;
vk(3,1)=vk_1(3,1)+ak_1(3,1)*T;
