function xiao_qiu_zhui_zong(ss,xun_huan,index)
ss(index(1,xun_huan),:)=[0, 0 ,0, 0 ,0 ];%删除矩阵ss的某一行% % %角度轨迹规划
for ge=1:1:10
    [xx1,yy1,zz1]=ellipsoid(ss(ge,1),ss(ge,2),ss(ge,3),ss(ge,5),ss(ge,5),ss(ge,5));
    surf(xx1,yy1,zz1); %半径为j的圆
    hold on
end

end