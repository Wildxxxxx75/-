% %Build Robot by D_H methods
%����DH������
global D5
ToDeg = 180/pi;%תΪ�Ƕ���
ToRad = pi/180;%תΪ������
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';
%J1Ϊ�ؽ����ƣ�
%thΪ��z����תʹx��ƽ�еĽǶ�
%dzΪ��z��ƽ��ʹx�Ṳ�ߵ��ƶ�����
%dxΪ��x��ƽ��ʹ�����ο�����ϵԭ���غϵ��ƶ�����
%alfΪ��x����תʹz���غϵĽǶ�
Link= struct('name','Body' , 'th',  0, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ);     % az 
Link(1)= struct('name','Base' , 'th',  0*ToRad, 'dz', -450, 'dx', -500, 'alf',0*ToRad,'az',UZ);        %Base To 1
Link(2) = struct('name','J1' , 'th',  90*ToRad, 'dz', 350, 'dx', 0, 'alf',90*ToRad,'az',UZ);       %1 TO 2
Link(3) = struct('name','J2' , 'th',  0*ToRad, 'dz', 0, 'dx', 400, 'alf',0*ToRad,'az',UZ);    %2 TO 3
Link(4) = struct('name','J3' , 'th',  0*ToRad, 'dz', 0, 'dx', 400, 'alf',0*ToRad,'az',UZ);          %3 TO 4
Link(5) = struct('name','J4' , 'th',  0*ToRad, 'dz', 0, 'dx', 400, 'alf',0*ToRad,'az',UZ);          %4 TO 5
Link(6) = struct('name','J5' , 'th',  0*ToRad, 'dz', 0, 'dx',400, 'alf',0*ToRad,'az',UZ);          %5 TO 6
Link(7) = struct('name','J6' , 'th',  90*ToRad, 'dz', 0, 'dx', 0, 'alf',-90*ToRad,'az',UZ);          %6 TO 
Link(8) = struct('name','J7' , 'th',  0*ToRad, 'dz', D5, 'dx', 0, 'alf',0*ToRad,'az',UZ);          %E TO E1


