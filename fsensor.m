%% fsensor 传感器，叠加噪声。

function [zbeitak,zebuxinonk,zDk]=fsensor(beitak,ebuxinonk,Dk,rbeitak,rebuxinonk,rDk)   

% beitak     方位角坐标
% ebuxinonk  高低角坐标
% Dk         距离坐标

% rbeitak    传感器方位角测量噪声标准偏差
% rebuxinonk 传感器高低角测量噪声标准偏差
% rDk        传感器距离测量噪声标准偏差

zbeitak=beitak+normrnd(0,rbeitak); 
zebuxinonk=ebuxinonk+normrnd(0,rebuxinonk);
zDk=Dk+normrnd(0,rDk);