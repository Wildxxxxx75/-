function Draw_line_5(z0,y0,y1,flag,ban_jing,ss,index,xun_huan,x0,x1)%ĩ��ֱ�ǹ滮
global p_x p_y p_z
global q_1 q_2 q_3 q_4 q_5 q_6 q_7
i=4;%���ڲ����������ı����ʱ��

Posion1=Creat_5_curve(0,y1-y0,0,0,0,0);%Ȼ�����y�����ƶ�
Posion2=Creat_5_curve(0,x1-x0,0,0,0,0);%�Ƚ���x�����ƶ�
n=length(Posion1);%����A��B��Ĳɼ����ݸ���
n1=length(p_x);

x(1)=0;y(1)=y0;z(1)=z0;

num=2;
for t=1:i:n%�ɼ����ݸ���Ϊn/i
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
%%%��ֵ��������

% plot3(p_x,p_y,p_z,'ko');hold on;%x,y,zΪ����
theta2=IK_7DOF_num_solu(W);%theta2Ϊ�Ƕ���
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
if(t>n-i)
    DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),0);
    [xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
% yan_se=ones(size(zz,1));
surf(xx,yy,zz); %�뾶Ϊj��Բ
xu_hao = num2str(xun_huan);%��ֵת�ַ�����׼��ѭ����1��10��10����
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%�����ţ�ԭ��ss�����е�m�е�1���������꣬
 %ss�����е�m�е�2����������Ȼ������š�
else
[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
% yan_se=ones(size(zz,1));
surf(xx,yy,zz); %�뾶Ϊj��Բ
xu_hao = num2str(xun_huan);%��ֵת�ַ�����׼��ѭ����1��10��10����
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%�����ţ�ԭ��ss�����е�m�е�1���������꣬
 %ss�����е�m�е�2����������Ȼ������š�
DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),1);%���

end

hold on
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
xiao_qiu_zhui_zong(ss,xun_huan,index);


% shading interp%��ֹsurf������ɫС��
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
% plot(t,q_1,'r','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q_2,'b','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q_3,'g','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q_4,'k','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q_5,'c','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q_6,'m','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q_7,'y','LineWidth',2);grid on;%���ƽǶ�λ��
% axis([1,num-1,-400,400]);%-180 180
end