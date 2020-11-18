syms x1 x2;
%优化函数
f = x1^2 + x2^2 + x1 + x2;
%梯度
g_x1 = diff(f, x1);
g_x2 = diff(f, x2);
%起始位置
x1_0 = -3;
x2_0 = -3;
%步长
step_ = 0.1;
fprintf("迭代步长为%.10f",step_);
% 梯度下降法
X=zeros(size(n));
Y=zeros(size(n));
Z=zeros(size(n));
tmpx = x1_0;
tmpx2 = x2_0;
dis = 1;
k = 1;
while ( dis > 0.01)
last_tmpf = subs(f,[x1,x2],[tmpx,tmpx2]);
g_x1 = subs(g_x1,[x1,x2],[tmpx,tmpx2]);
g_x2 = subs(g_x2,[x1,x2],[tmpx,tmpx2]);
tmpx = tmpx - step_*g_x1;
tmpx2 =tmpx2 - step_*g_x2;
tmpf = subs(f,[x1,x2],[tmpx,tmpx2]);
X(k) = tmpx;
Y(k) = tmpx2;
Z(k) = tmpf;
dis = abs(tmpf -last_tmpf);
k = k + 1;
if(k >= 2000)
fprintf("梯度下降法迭代了%d次还没收敛",k);
break;
end
end
if(k < 2000)
fprintf("梯度下降法迭代次数为%d",k-1);
end
% 半隐
X1=zeros(size(n));
Y1=zeros(size(n));
Z1=zeros(size(n));
tmpx = x1_0;
tmpx2 = x2_0;
dis = 1;
k = 1;
while ( dis > 0.01)
last_tmpf = subs(f,[x1,x2],[tmpx,tmpx2]);
tmpx = (tmpx - step_)/(2*step_+1);
tmpx2 = (tmpx2 - step_)/(2*step_+1);
tmpf = subs(f,[x1,x2],[tmpx,tmpx2]);
X1(k) = tmpx;
Y1(k) = tmpx2;
Z1(k) = tmpf;
dis = abs(tmpf -last_tmpf);
k = k + 1;
if(k >= 2000)
fprintf("半隐后迭代了%d次还没收敛",k);
break;
end
end
if ( k < 2000)
fprintf("半隐后迭代次数为%d",k-1);
end
scatter3(X,Y,Z,'g');
hold on;
scatter3(X1,Y1,Z1,'r');
grid on;