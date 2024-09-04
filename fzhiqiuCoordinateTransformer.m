%% fzhiqiuCoordinateTransformer 直/球坐标变换
function [beitak,ebuxinonk,Dk]=fzhiqiuCoordinateTransformer(xk)

% beitak     方位角坐标
% ebuxinonk  高低角坐标
% Dk         距离坐标

% xk         直角坐标向量

% 航迹真值生成—球坐标
beitak=atan2(xk(2,1),xk(1,1)); % atan2四象限反正切
ebuxinonk=atan2(xk(3,1),(xk(1,1)^2+xk(2,1)^2)^0.5);
Dk=(xk(1,1)^2+xk(2,1)^2+xk(3,1)^2)^0.5;


