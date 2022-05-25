% clc;
% clear;
% close all;
function rrt_SanWei(ss,index,xun_huan,ban_jing)
%ss是主函数中的存储小球信息的矩阵，index将小球体积从大到小排列后的矩阵
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);%绘制水平侧卧的圆柱
hold on
circleCenter = [ss(1,1),ss(1,2),ss(1,3);ss(2,1),ss(2,2),ss(2,3);ss(3,1),ss(3,2),ss(3,3);ss(4,1),ss(4,2),ss(4,3);ss(5,1),ss(5,2),ss(5,3);ss(6,1),ss(6,2),ss(6,3);ss(7,1),ss(7,2),ss(7,3);ss(8,1),ss(8,2),ss(8,3);ss(9,1),ss(9,2),ss(9,3);ss(10,1),ss(10,2),ss(10,3)];
%将10个小球的x，y，z坐标输入
r=[ss(1,5);ss(2,5);ss(3,5);ss(4,5);ss(5,5);ss(6,5);ss(7,5);ss(8,5);ss(9,5);ss(10,5)];
%输入10个小球的半径
for ii=1:1:10
[xx,yy,zz]= ellipsoid(ss(ii,1),ss(ii,2),ss(ii,3),ss(ii,5),ss(ii,5),ss(ii,5));%绘制椭圆函数，半径一样时就变成了圆
surf(xx,yy,zz); %半径为j的圆
% xu_hao = num2str(xun_huan);%数值转字符串，准备循环从1到10，10个数
% text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%标记序号，原来ss矩阵中第m行第1列做横坐标，
%  %ss矩阵中第m行第2列做纵坐标然后标记序号。
hold on
end

%% 起点source的坐标，目标点goal的坐标
source=[ss(index(1,xun_huan),1)-ss(index(1,xun_huan),5)-2 ss(index(1,xun_huan),2) ss(index(1,xun_huan),3)];
goal=[0 ss(index(1,xun_huan),2) ss(index(1,xun_huan),3)];

stepsize = 40;%RRT每一步的步长
threshold = 20;%界
maxFailedAttempts = 1000;%最大生长次数
display = true;
searchSize =[ -1550 100 100];      %给定生长方向，

%% 缁樺埗璧风偣鍜岀粓鐐?

% scatter3(source(1),source(2),source(3),'filled','g');
% scatter3(goal(1),goal(2),goal(3),'filled','b');%绘制生长轨迹
% hold on
tic;  % tic-toc: 记录当前运行时间
RRTree = double([source -1]);
failedAttempts = 0;
pathFound = false;

%% 寰幆
while failedAttempts <= maxFailedAttempts  % loop to grow RRTs
    %% chooses a random configuration
    if rand <0.6    %(50%随机点，50%为目标点) rand是随机产生一个0到1内的均匀随机分布数
        sample = rand(1,3).* searchSize;   % random sample rand（1，3）产生1*3的矩阵, .*是将矩阵对应位置相乘
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
%绘制生长成功后的路径轨迹
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
%获取机器人末端位置
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
theta2=IK_7DOF_num_solu(W);%theta2为角度制
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %半径为j的圆
DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),0);
[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %半径为j的圆
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),1);

shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
xiao_qiu_zhui_zong(ss,xun_huan,index);
hold on
% plot3(x(num),y[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
% yan_se=ones(size(zz,1));
% surf(xx,yy,zz); %半径为j的圆
xu_hao = num2str(xun_huan);%数值转字符串，准备循环从1到10，10个数
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%标记序号，原来ss矩阵中第m行第1列做横坐标，
 %ss矩阵中第m行第2列做纵坐标然后标记序号。(num),z(num),'ko');%x,y,z为数组
 view(64,20)
q1(num)=theta2(1);
q2(num)=theta2(2);
q3(num)=theta2(3);
q4(num)=theta2(4);
q5(num)=theta2(5);
q6(num)=theta2(6);
q7(num)=theta2(7);

Angel=[Angel;q1(num),q2(num),q3(num),q4(num),q5(num),q6(num),q7(num)]%画关节的速度，加速度等
num=num+1;
hold on
end
%RRT_zhi_biao(Angel);%绘制RRT避障关节轴指标

end
