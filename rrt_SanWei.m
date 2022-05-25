% clc;
% clear;
% close all;
function rrt_SanWei(ss,index,xun_huan,ban_jing)
%ss���������еĴ洢С����Ϣ�ľ���index��С������Ӵ�С���к�ľ���
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);%����ˮƽ���Ե�Բ��
hold on
circleCenter = [ss(1,1),ss(1,2),ss(1,3);ss(2,1),ss(2,2),ss(2,3);ss(3,1),ss(3,2),ss(3,3);ss(4,1),ss(4,2),ss(4,3);ss(5,1),ss(5,2),ss(5,3);ss(6,1),ss(6,2),ss(6,3);ss(7,1),ss(7,2),ss(7,3);ss(8,1),ss(8,2),ss(8,3);ss(9,1),ss(9,2),ss(9,3);ss(10,1),ss(10,2),ss(10,3)];
%��10��С���x��y��z��������
r=[ss(1,5);ss(2,5);ss(3,5);ss(4,5);ss(5,5);ss(6,5);ss(7,5);ss(8,5);ss(9,5);ss(10,5)];
%����10��С��İ뾶
for ii=1:1:10
[xx,yy,zz]= ellipsoid(ss(ii,1),ss(ii,2),ss(ii,3),ss(ii,5),ss(ii,5),ss(ii,5));%������Բ�������뾶һ��ʱ�ͱ����Բ
surf(xx,yy,zz); %�뾶Ϊj��Բ
% xu_hao = num2str(xun_huan);%��ֵת�ַ�����׼��ѭ����1��10��10����
% text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%�����ţ�ԭ��ss�����е�m�е�1���������꣬
%  %ss�����е�m�е�2����������Ȼ������š�
hold on
end

%% ���source�����꣬Ŀ���goal������
source=[ss(index(1,xun_huan),1)-ss(index(1,xun_huan),5)-2 ss(index(1,xun_huan),2) ss(index(1,xun_huan),3)];
goal=[0 ss(index(1,xun_huan),2) ss(index(1,xun_huan),3)];

stepsize = 40;%RRTÿһ���Ĳ���
threshold = 20;%��
maxFailedAttempts = 1000;%�����������
display = true;
searchSize =[ -1550 100 100];      %������������

%% 绘制起点和终�?

% scatter3(source(1),source(2),source(3),'filled','g');
% scatter3(goal(1),goal(2),goal(3),'filled','b');%���������켣
% hold on
tic;  % tic-toc: ��¼��ǰ����ʱ��
RRTree = double([source -1]);
failedAttempts = 0;
pathFound = false;

%% 循环
while failedAttempts <= maxFailedAttempts  % loop to grow RRTs
    %% chooses a random configuration
    if rand <0.6    %(50%����㣬50%ΪĿ���) rand���������һ��0��1�ڵľ�������ֲ���
        sample = rand(1,3).* searchSize;   % random sample rand��1��3������1*3�ľ���, .*�ǽ������Ӧλ�����
    else
        sample = goal; % sample taken as goal to bias tree generation to goal
    end
    %% selects the node in the RRT tree that is closest to qrand

[A, I] = min( distanceCost(RRTree(:,1:3),sample) ,[],1); % find the minimum value of each column

    closestNode = RRTree(I(1),1:3);
    %% moving from qnearest an incremental distance in the direction of qrand
    movingVec = [sample(1)-closestNode(1),sample(2)-closestNode(2),sample(3)-closestNode(3)];
    movingVec = movingVec/sqrt(sum(movingVec.^2));  
    newPoint = closestNode + stepsize * movingVec;
    if ~checkPath3(closestNode, newPoint, circleCenter,r) % if extension of closest node in tree to the new point is feasible
        failedAttempts = failedAttempts + 1;
        continue;
    end
    
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end % goal reached
    [A, I2] = min( distanceCost(RRTree(:,1:3),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree(I2(1),1:3)) < threshold, failedAttempts = failedAttempts + 1; continue; end 
    
    RRTree = [RRTree; newPoint I(1)]; % add node
    failedAttempts = 0;
%     if display, plot3([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'LineWidth',1); end
%���������ɹ����·���켣
%     pause(0.05);
end


%% retrieve path from parent information
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
path = goal;
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:3); path];
    prev = RRTree(prev,4);
end

pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + distanceCost(path(i,1:3),path(i+1,1:3)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 

hold on;

hang_shu=size(path,1);
%��ȡ������ĩ��λ��
num=1;
Angel=[];
for hang=1:1:hang_shu
figure(1)
x(num)=path(hang,1);
y(num)=path(hang,2);
z(num)=path(hang,3);
W=[1,0,0,x(num);
       0,1,0,y(num);
       0,0,1,z(num);
       0,0,0,1];
theta2=IK_7DOF_num_solu(W);%theta2Ϊ�Ƕ���
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %�뾶Ϊj��Բ
DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),0);
[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %�뾶Ϊj��Բ
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),1);

shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
xiao_qiu_zhui_zong(ss,xun_huan,index);
hold on
% plot3(x(num),y[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
% yan_se=ones(size(zz,1));
% surf(xx,yy,zz); %�뾶Ϊj��Բ
xu_hao = num2str(xun_huan);%��ֵת�ַ�����׼��ѭ����1��10��10����
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%�����ţ�ԭ��ss�����е�m�е�1���������꣬
 %ss�����е�m�е�2����������Ȼ������š�(num),z(num),'ko');%x,y,zΪ����
 view(64,20)
q1(num)=theta2(1);
q2(num)=theta2(2);
q3(num)=theta2(3);
q4(num)=theta2(4);
q5(num)=theta2(5);
q6(num)=theta2(6);
q7(num)=theta2(7);

Angel=[Angel;q1(num),q2(num),q3(num),q4(num),q5(num),q6(num),q7(num)]%���ؽڵ��ٶȣ����ٶȵ�
num=num+1;
hold on
end
%RRT_zhi_biao(Angel);%����RRT���Ϲؽ���ָ��

end
