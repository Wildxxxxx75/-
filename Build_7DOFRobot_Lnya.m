% %Build Robot by D_H methods
%建立DH参数表
global D5
ToDeg = 180/pi;%转为角度制
ToRad = pi/180;%转为弧度制
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';
%J1为关节名称，
%th为绕z轴旋转使x轴平行的角度
%dz为沿z轴平移使x轴共线的移动距离
%dx为沿x轴平移使两个参考坐标系原点重合的移动距离
%alf为绕x轴旋转使z轴重合的角度
Link= struct('name','Body' , 'th',  0, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ);     % az 
Link(1)= struct('name','Base' , 'th',  0*ToRad, 'dz', -450, 'dx', -500, 'alf',0*ToRad,'az',UZ);        %Base To 1
Link(2) = struct('name','J1' , 'th',  90*ToRad, 'dz', 350, 'dx', 0, 'alf',90*ToRad,'az',UZ);       %1 TO 2
Link(3) = struct('name','J2' , 'th',  0*ToRad, 'dz', 0, 'dx', 400, 'alf',0*ToRad,'az',UZ);    %2 TO 3
Link(4) = struct('name','J3' , 'th',  0*ToRad, 'dz', 0, 'dx', 400, 'alf',0*ToRad,'az',UZ);          %3 TO 4
Link(5) = struct('name','J4' , 'th',  0*ToRad, 'dz', 0, 'dx', 400, 'alf',0*ToRad,'az',UZ);          %4 TO 5
Link(6) = struct('name','J5' , 'th',  0*ToRad, 'dz', 0, 'dx',400, 'alf',0*ToRad,'az',UZ);          %5 TO 6
Link(7) = struct('name','J6' , 'th',  90*ToRad, 'dz', 0, 'dx', 0, 'alf',-90*ToRad,'az',UZ);          %6 TO 
Link(8) = struct('name','J7' , 'th',  0*ToRad, 'dz', D5, 'dx', 0, 'alf',0*ToRad,'az',UZ);          %E TO E1


