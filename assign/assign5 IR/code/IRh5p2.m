%Algolrithm motion_model_odometry with Normal Distribution Noise
clc
close all
%Initial setting of mobile robot
x =100;
y =100;
theta = pi/4;

a1 = 0.000005;
a2 = 0.000005;
a3 = 0.000005;
a4 = 0.000005;

trajectory_data = zeros(3,500,30);
odom = zeros(3,30);
odom(:,:) = NaN;
odom(:,1:3)= 0;

trajectory_data(:,:,:) = NaN;
trajectory_data(:,:,1) = 0;

n = 1;
t = 2;

while (t <= 30 )
    
    
    if t < 10
        delta_rot1 = 0;
        
        delta_trans =50;
        
        delta_rot2 = 0;
        
        
    elseif (t >= 10)&&(t < 12)
        delta_rot1 = 0;
        
        delta_trans = 50;
        
        delta_rot2 = pi/4;
        
        
        
        
    elseif (t >= 12)&&(t < 20)
        delta_rot1 = 0;
        
        delta_trans = 50;
        
        delta_rot2 = 0;
        
        
        
        
    elseif (t >= 20)&&(t < 22)
        delta_rot1 = 0;
        
        delta_trans = 50;
        
        delta_rot2 = pi/4;
        
        
    elseif (t >= 22)&&(t <= 30)
        delta_rot1 = 0;
        
        delta_trans = 50;
        
        delta_rot2 = 0;
        
        
    end
    
    
    for n = 1: 500
        
        X = sample_motion_model([delta_rot1,delta_rot2,delta_trans],trajectory_data(:,n,t-1),theta,a1,a2,a3,a4);
        
        trajectory_data(1,n,t) = X(1);
        
        trajectory_data(2,n,t) = X(2);
        
        trajectory_data(3,n,t) = X(3);
        
        theta = X(3);
        
    end
    
    
    
    t = t + 1;
    
    
    if t < 10
        
        delta_rot1 = 0;
        
        delta_trans =50;
        
        delta_rot2 = 0;
        
        odom(1,t) = odom(1,t-1) + delta_trans;
        
        odom(2,t) = odom(2,t-1);
        
        odom(3,t) = odom(3,t-1) + delta_rot1 + delta_rot2;
        
    elseif (t >= 10)&&(t < 12)
        delta_rot1 = 0;
        
        delta_trans = 50;
        
        delta_rot2 = deg2rad(45);
        
        odom(1,t) = odom(1,t-1) + delta_trans * cos(theta + delta_rot1);
        
        odom(2,t) = odom(2,t-1) + delta_trans * sin(theta + delta_rot1);
        
        odom(3,t) = odom(3,t-1) + delta_rot1 + delta_rot2;
        
        
    elseif (t >= 12)&&(t < 20)
        delta_rot1 = 0;
        
        delta_trans = 50;
        
        delta_rot2 = 0;
        
        odom(1,t) = odom(1,t-1);
        
        odom(2,t) = odom(2,t-1) + delta_trans;
        
        odom(3,t) = odom(3,t-1) + delta_rot1 + delta_rot2;
        
        
    elseif (t >= 20)&&(t < 22)
        delta_rot1 = 0;
        
        delta_trans = 50;
        
        delta_rot2 = deg2rad(45);
        
        odom(1,t) = odom(1,t-1) + delta_trans * cos(theta + delta_rot1);
        
        odom(2,t) = odom(2,t-1) + delta_trans * sin(theta + delta_rot1);
        
        odom(3,t) = odom(3,t-1) + delta_rot1 + delta_rot2;
        
        
    elseif (t >= 22)&&(t <= 31)
        delta_rot1 = 0;
        
        delta_trans = 50;
        
        delta_rot2 = 0;
        
        odom(1,t) = odom(1,t-1) + delta_trans * cos(-pi);
        
        odom(2,t) = odom(2,t-1);
        
        odom(3,t) = odom(3,t-1) + delta_rot1 + delta_rot2;
    end
end

plot(odom(1,:),odom(2,:),'r','LineWidth',1.5);
hold on

for m = 1:30
    scatter(trajectory_data(1,5:500,m),trajectory_data(2,5:500,m),'.');
    hold on
end

function result = sample_motion_model(u,X,theta,a1,a2,a3,a4)
delta_rot1_noise = u(1) - normrnd(0,(a1*u(1)^2 + a2*u(3)^2));
delta_trans_noise = u(3) - normrnd(0,(a3*u(3)^2 + a4*u(1)^2 + a4*u(2)^2));
delta_rot2_noise = u(2) - normrnd(0,(a1*u(2)^2 + a2*u(3)^2));

x = X(1) + delta_trans_noise * cos(theta + delta_rot1_noise);
y = X(2) + delta_trans_noise * sin(theta + delta_rot1_noise);
theta = X(3) + delta_rot1_noise + delta_rot2_noise;
result = [x,y,theta];
end