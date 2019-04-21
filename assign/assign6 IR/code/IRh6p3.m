clc
close all
clear all

num = 1000;
map = [0 0;5 0;5 10];
x0 = 9;
y0 = 5;

[n,m] = size(map);
d_star = (map(:,1)-x0).^2+(map(:,2)-y0).^2;
p = zeros(num,2);
for i = 1:num
d = d_star+randn(3)-1;
[X,fval] = fsolve(@(X) get_p(X,map,d),[0,0]);
p(i,1) = X(1);
p(i,2) = X(2);
end

hold on
scatter(map(:,1),map(:,2));
scatter(p(:,1),p(:,2),'.','b');
scatter(x0,y0,'red');


function f = get_p(p,map,d)
f = [(map(1,1)-p(1))^2+(map(1,2)-p(2))^2-d(1);
    (map(2,1)-p(1))^2+(map(2,2)-p(2))^2-d(2);
    (map(3,1)-p(1))^2+(map(3,2)-p(2))^2-d(3)];
end