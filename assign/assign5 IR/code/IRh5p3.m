clc
close all

u = [1 0];
%task 1
%a = [0.02 0.02 0.2 0.2 0.005 0.005];

%task 2
% a = [0.03 0.03 0.03 0.03 0.005 0.005];

%task 3
a = [0.005 0.005 0.5 0.5 0.005 0.005];

X0 = [0 0 pi/4];
X = zeros(3,500);

d_t = 0.5;
for i=1:500
    X(:,i) = sample_motion_model_velocity(u,X0,a,d_t);
end
hold on
scatter(X0(1),X0(2));
scatter(X(1,:),X(2,:),'.');
hold off
function result = sample_motion_model_velocity(u,X,a,d_t)
v = u(1)+normrnd(0,(a(1)*u(1)^2+a(2)*u(2)^2));
w = u(2)+normrnd(0,(a(3)*u(1)^2+a(4)*u(2)^2));
gamma = normrnd(0,(a(5)*u(1)^2+a(6)*u(2)^2));

x = X(1)-v/w*sin(X(3))+v/w*sin(X(3)+w*d_t);
y = X(2)+v/w*cos(X(3))-v/w*cos(X(3)+w*d_t);
theta = X(3)+w*d_t+gamma*d_t;
result = [x,y,theta];
end
    