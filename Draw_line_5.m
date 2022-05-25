function Draw_line_5(z0,y0,y1,flag,ban_jing,ss,index,xun_huan,x0,x1)%末端直角规划
global p_x p_y p_z
global q_1 q_2 q_3 q_4 q_5 q_6 q_7
i=4;%调节采样数量来改变绘制时间

Posion1=Creat_5_curve(0,y1-y0,0,0,0,0);%然后进行y方向移动
Posion2=Creat_5_curve(0,x1-x0,0,0,0,0);%先进行x方向移动
n=length(Posion1);%设置A到B点的采集数据个数
n1=length(p_x);

x(1)=0;y(1)=y0;z(1)=z0;

num=2;
for t=1:i:n%采集数据个数为n/i
%     x(num)=0;p_x(n1+num)=0;
    z(num)=z0;p_z(n1+num)=z0;
    if(flag==1) 
%         z(num)=z(num-1)+Posion1(t);
      y(num)=y0+Posion1(t);
    else if(flag==0)
        y(num)=y0-Posion1(t); 
       
    else if(flag==3)
            x(num)=x0+Posion2(t);
        end
        end
    end
    if(flag==1)
    p_y(n1+num)=y(num);
    x(num)=0;p_x(n1+num)=0;
    
    else if(flag==3)
   p_x(n1+num)=x(num);
   y(num)=y0;p_y(n1+num)=y0;
        end
    end
    

    W=[1,0,0,x(num);
       0,1,0,y(num);
       0,0,1,z(num);
       0,0,0,1];
%%%数值解计算逆解

% plot3(p_x,p_y,p_z,'ko');hold on;%x,y,z为数组
theta2=IK_7DOF_num_solu(W);%theta2为角度制
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
if(t>n-i)
    DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),0);
    [xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
% yan_se=ones(size(zz,1));
surf(xx,yy,zz); %半径为j的圆
xu_hao = num2str(xun_huan);%数值转字符串，准备循环从1到10，10个数
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%标记序号，原来ss矩阵中第m行第1列做横坐标，
 %ss矩阵中第m行第2列做纵坐标然后标记序号。
else
[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
% yan_se=ones(size(zz,1));
surf(xx,yy,zz); %半径为j的圆
xu_hao = num2str(xun_huan);%数值转字符串，准备循环从1到10，10个数
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%标记序号，原来ss矩阵中第m行第1列做横坐标，
 %ss矩阵中第m行第2列做纵坐标然后标记序号。
DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),1);%清除

end

hold on
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
xiao_qiu_zhui_zong(ss,xun_huan,index);


% shading interp%防止surf画出黑色小球
q_1(num)=theta2(1);
q_2(num)=theta2(2);
q_3(num)=theta2(3);
q_4(num)=theta2(4);
q_5(num)=theta2(5);
q_6(num)=theta2(6);
q_7(num)=theta2(7);
num=num+1;
end

% t=1:1:num-1;
% figure(flag+1),
% plot(t,q_1,'r','LineWidth',2);hold on;%绘制角度位置
% plot(t,q_2,'b','LineWidth',2);hold on;%绘制角度位置
% plot(t,q_3,'g','LineWidth',2);hold on;%绘制角度位置
% plot(t,q_4,'k','LineWidth',2);hold on;%绘制角度位置
% plot(t,q_5,'c','LineWidth',2);hold on;%绘制角度位置
% plot(t,q_6,'m','LineWidth',2);hold on;%绘制角度位置
% plot(t,q_7,'y','LineWidth',2);grid on;%绘制角度位置
% axis([1,num-1,-400,400]);%-180 180
end