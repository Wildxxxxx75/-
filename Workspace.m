close all;

clear;

global Link
Build_7DOFRobot_Lnya;
ss=[];%�洢С����Ϣ�ľ���
%ellipsoid(x,y,z,x1,y1,z1) x,y,z���������������,x1,y1,z1������x,y,z����ķ���
for ii=1:1:10
   BanJing=randi([25 40]);%����һ��25��40֮��������������Բ�İ뾶
%  j=unidrnd(50);%����һ��50���ڵ������
   
   s1=unidrnd(50);%s���������С��ĳ�ʼ��
   s2=randi([50 150]);%s���������С��ĳ�ʼ��
   s3=randi([100 157]);%s���������С��ĳ�ʼ��
%    s4=randsrc();%�����������1
   %�������С��λ�ã�����ÿ��С��Բ������a,b,c�������60����֤ÿ����С������С��200
   aa=s1+100+ii*80;
%     aa=0;
%    bb=s4*s2+ii*60;
%    ccc=s4*s3+ii*60;
    bb=s2+ii*70;
    pp=s3+ii*70;
   [xx,yy,zz]= ellipsoid(aa,bb,pp,BanJing,BanJing,BanJing);
   surf(xx,yy,zz) %�뾶Ϊj��Բ
%    shading interp%��ֹsurf������ɫС��
   dd=(4/3)*pi*BanJing^3;%������
   ti_ji=round(dd);%�����ȡ��
   ss=[ss;aa,bb,pp,ti_ji,BanJing];
%    yan_se=ones(size(zz,1));
  
   disp(ss)%��ӡ��ÿ��С���Բ����������
   hold on
%    view(-8,38);%����ڣ�-8 ��38���Ƕȹۿ�
   alpha(0.5)%����ϵ������ͼƬ͸����Ϊ0.5��1�ǲ�͸�� 
end


ToDeg = 180/pi;
ToRad = pi/180;

num=1;

for th1=-180:60:180%ÿ�����
    for th2=-180:10:180
        for th3=-245:100:65
            for th4=-200:100:200
                for th5=-115:46:115
                    for th6=-400:100:400
                        for th7=-360:360:360
                        theta1=th1*ToRad;
                        theta2=th2*ToRad;
                        theta3=th3*ToRad;
                        theta4=th4*ToRad;
                        theta5=th5*ToRad;
                        theta6=th6*ToRad;
                        theta7=th7*ToRad;

                        A1 =[[ cos(theta1), 0 ,-sin(theta1) ,   0]
                                [ sin(theta1),  0,cos(theta1),   0]
                                [           0, -1,            0, 350]
                                [           0,  0,            0,   1]];
                        A2 =[[ cos(theta2 ), -sin(theta2),0, 400*cos(theta2)]
                                [ sin(theta2 ),cos(theta2),0,400*sin(theta2)]
                                [     0,     1, 0, 0]
                                [        0,       0, 0,   1]];
                        A3 =[[ cos(theta3), -sin(theta3),0,400*cos(theta3)]
                                [ sin(theta3), cos(theta3),0,400*sin(theta3)]
                                [           0,   0,   1,     0]
                                [           0,  0,            0,   1]];
                        A4 =[[ cos(theta4), -sin(theta4),0,400*cos(theta4)]
                                [ sin(theta4), cos(theta4),0,400*sin(theta4)]
                                [           0,   0,   1,     0]
                                [           0,  0,            0,   1]];
                        A5 =[[ cos(theta5), -sin(theta5),0, 400*cos(theta5)]
                                [ sin(theta5), cos(theta5), 0,400*sin(theta5)]
                                [           0,   0,   1,     0]
                                [           0,  0,0,   1]];
                        A6 =[[ cos(theta6), 0, -sin(theta6), 0]
                                [ sin(theta6),  0, cos(theta6), 0]
                                [  0,  -1, 0,             0]
                                [    0,         0, 0,           1]];
                        A7 =[[cos(theta7), -sin(theta7), 0,0]
                                [ sin(theta7), cos(theta7),0, 0]
                                [  0,  0, 0,    170]
                                [    0,         0, 0,           1]];
                     
                        A = A1 * A2 * A3 * A4 * A5 * A6*A7;
            
                        point1(num) = A(1,4);
                        point2(num) = A(2,4);
                        point3(num) = A(3,4);
                        num = num + 1;  
                       end
                   end
               end
           end
       end
    end
end
plot3(point1,point2,point3,'r*');
axis([-3000,3000,-3000,3000,-3000,3000]);
grid on;

