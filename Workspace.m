close all;

clear;

global Link
Build_7DOFRobot_Lnya;
ss=[];%存储小球信息的矩阵
%ellipsoid(x,y,z,x1,y1,z1) x,y,z代表了椭球的中心,x1,y1,z1代表了x,y,z方向的分量
for ii=1:1:10
   BanJing=randi([25 40]);%产生一个25到40之间的随机数，当作圆的半径
%  j=unidrnd(50);%产生一个50以内的随机数
   
   s1=unidrnd(50);%s是随机生成小球的初始点
   s2=randi([50 150]);%s是随机生成小球的初始点
   s3=randi([100 157]);%s是随机生成小球的初始点
%    s4=randsrc();%随机生成正负1
   %随机生成小球位置，并且每个小球圆心坐标a,b,c至少相差60，保证每两个小球间相距小于200
   aa=s1+100+ii*80;
%     aa=0;
%    bb=s4*s2+ii*60;
%    ccc=s4*s3+ii*60;
    bb=s2+ii*70;
    pp=s3+ii*70;
   [xx,yy,zz]= ellipsoid(aa,bb,pp,BanJing,BanJing,BanJing);
   surf(xx,yy,zz) %半径为j的圆
%    shading interp%防止surf画出黑色小球
   dd=(4/3)*pi*BanJing^3;%球的体积
   ti_ji=round(dd);%对体积取整
   ss=[ss;aa,bb,pp,ti_ji,BanJing];
%    yan_se=ones(size(zz,1));
  
   disp(ss)%打印出每个小球的圆心坐标和体积
   hold on
%    view(-8,38);%最后在（-8 ，38）角度观看
   alpha(0.5)%坐标系内所有图片透明度为0.5，1是不透明 
end


ToDeg = 180/pi;
ToRad = pi/180;

num=1;

for th1=-180:60:180%每个轴的
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

