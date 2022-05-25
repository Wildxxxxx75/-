function pic=DHfk7Dof_Lnya(th1,th2,th3,th4,th5,th6,th7,fcla)
%%%���ƻ�����
% close all

global Link
Build_7DOFRobot_Lnya;%����DH������
%��ͼ���ò���
radius    = 30;
len       = 90;
joint_col = 0;
% plot3(0,0,0,'ro'); 
%����Ƕ�ת��Ϊ����
 Link(2).th=th1*pi/180;
 Link(3).th=th2*pi/180;
 Link(4).th=th3*pi/180;
 
 Link(5).th=th4*pi/180;
 Link(6).th=th5*pi/180;
 Link(7).th=th6*pi/180;
 Link(8).dz=th7;%ĩ�˻����ؽ�
 %%

%����DH�������ؽڱ�����������ؽڵ���α任����
for i=1:8
Matrix_DH_Ln(i);
end
%���Ʊ�ʾ�����ؽڵ�Բ���弰�ؽ�����
for i=2:8
      %�����i���ؽڵ�λ�ˣ�������һ�ؽڵ�λ�˼�����һ�ؽڵ�λ��
      Link(i).A=Link(i-1).A*Link(i).A;
      Link(i).p= Link(i).A(:,4);
      Link(i).n= Link(i).A(:,1);
      Link(i).o= Link(i).A(:,2);
      Link(i).a= Link(i).A(:,3);
      Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
      %���Ƶ�i-1���ؽ�
      Connect3D(Link(i-1).p,Link(i).p,'b',2); hold on;
%       plot3(Link(i).p(1),Link(i).p(2),Link(i).p(3),'rx');hold on;
      DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on;
end

    

grid on;
% view(134,12);

axis([-2500,2500,-2500,2500,-2500,2500]);


xlabel('x');
ylabel('y');
zlabel('z');
drawnow;
pic=getframe;%��������������ͼ����ӰƬ֡
if(fcla)
    cla;
end


