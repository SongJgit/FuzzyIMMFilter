%% fzhiqiuCoordinateTransformer ֱ/������任
function [beitak,ebuxinonk,Dk]=fzhiqiuCoordinateTransformer(xk)

% beitak     ��λ������
% ebuxinonk  �ߵͽ�����
% Dk         ��������

% xk         ֱ����������

% ������ֵ���ɡ�������
beitak=atan2(xk(2,1),xk(1,1)); % atan2�����޷�����
ebuxinonk=atan2(xk(3,1),(xk(1,1)^2+xk(2,1)^2)^0.5);
Dk=(xk(1,1)^2+xk(2,1)^2+xk(3,1)^2)^0.5;


