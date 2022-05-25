function J=Jacobian7DoF_Ln(th1,th2,th3,th4,th5,th6,th7)
% close all

global Link

jsize=7;
J=zeros(6,jsize);

Link(2).th=th1*pi/180;
Link(3).th=th2*pi/180;
Link(4).th=th3*pi/180;
Link(5).th=th4*pi/180;
Link(6).th=th5*pi/180;
Link(7).th=th6*pi/180;
Link(8).dz=th7;
for i=1:8
Matrix_DH_Ln(i);
end

Link(1).p=Link(1).p(1:3);
for i=2:8
      Link(i).A=Link(i-1).A*Link(i).A;
      Link(i).p= Link(i).A(1:3,4);
      Link(i).n= Link(i).A(:,1);
      Link(i).o= Link(i).A(:,2);
      Link(i).a= Link(i).A(:,3);
      Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
end

target=Link(8).p;

for n=1:jsize
     a=Link(n).R*Link(n).az;  
     if(n==7) 
         b=[0,0,0]';
         J(:,n)=[a; b];
     else
     J(:,n)=[cross(a,target-Link(n).p); a];
     end
end



