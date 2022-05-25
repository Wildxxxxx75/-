function A=DHfk7Dof_kuka_nodraw(q)
global Link
Build_7DOFRobot_Lnya;%建立DH参数表

%input angle
Link(2).th = q(1)*pi/180;
Link(3).th = q(2)*pi/180;
Link(4).th = q(3)*pi/180;
Link(5).th = q(4)*pi/180;
Link(6).th = q(5)*pi/180;
Link(7).th = q(6)*pi/180;

Link(8).dz = q(7);
%DH matrix
for i=1:8
    Matrix_DH_Ln(i);
end

%position and orientation
for i=2:8
	Link(i).A=Link(i-1).A*Link(i).A;
    Link(i).p= Link(i).A(:,4);
    Link(i).n= Link(i).A(:,1);
    Link(i).o= Link(i).A(:,2);
    Link(i).a= Link(i).A(:,3);
    Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
end

A=Link(8).A;%4*4 的矩阵
