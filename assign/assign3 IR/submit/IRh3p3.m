close all;
clear all;


SatellitePosition=[19394,20003,49385;-20065,-6051,51085;1089,-6,48512;4056,-234,49756];
ideal_user_position = [30, 40, 50];

% the speed of light
c = 3*10^8;

% the synchronized sender's time
usr_time = 1000;

d = sqrt((SatellitePosition(:,1)-ideal_user_position(1)).^2+(SatellitePosition(:,2)-ideal_user_position(2)).^2+(SatellitePosition(:,3)-ideal_user_position(3)).^2);


syn_t = -d/c+usr_time;
disp(syn_t)


[p,fval] = fsolve(@(p) getSyn(p,SatellitePosition(:,1),SatellitePosition(:,2),SatellitePosition(:,3),syn_t,c,usr_time),ideal_user_position);

asyn_t = syn_t;

[a_p,fval] = fsolve(@(a_p) getAsyn(a_p,SatellitePosition(:,1),SatellitePosition(:,2),SatellitePosition(:,3),asyn_t,c,usr_time),[ideal_user_position,0]);

disp('the calculated synchronized user position is ');
disp(p');
disp(a_p');

disp('the ideal user position is ');
disp(ideal_user_position');

scatter3(SatellitePosition(:,1),SatellitePosition(:,2),SatellitePosition(:,3));
hold on
scatter3(ideal_user_position(:,1),ideal_user_position(:,2),ideal_user_position(:,3));
scatter3(p(1),p(2),p(3));

for i=1:length(SatellitePosition)
    plot3([SatellitePosition(i,1),ideal_user_position(1)],[SatellitePosition(i,2),ideal_user_position(2)],[SatellitePosition(i,3),ideal_user_position(3)]);
    plot3([SatellitePosition(i,1),p(1)],[SatellitePosition(i,2),p(2)],[SatellitePosition(i,3),p(3)]);

end


function f = getSyn(p,x,y,z,t,c,usr_time)
f = [sqrt((x(1)-p(1))^2+(y(1)-p(2))^2+(z(1)-p(3))^2)-c*(usr_time-t(1));
    sqrt((x(2)-p(1))^2+(y(2)-p(2))^2+(z(2)-p(3))^2)-c*(usr_time-t(2));
    sqrt((x(3)-p(1))^2+(y(3)-p(2))^2+(z(3)-p(3))^2)-c*(usr_time-t(3))];
end

function f = getAsyn(a_p,x,y,z,t,c,usr_time)
f = [sqrt((x(1)-a_p(1))^2+(y(1)-a_p(2))^2+(z(1)-a_p(3))^2)-c*(usr_time+a_p(4)-t(1));
    sqrt((x(2)-a_p(1))^2+(y(2)-a_p(2))^2+(z(2)-a_p(3))^2)-c*(usr_time+a_p(4)-t(2));
    sqrt((x(3)-a_p(1))^2+(y(3)-a_p(2))^2+(z(3)-a_p(3))^2)-c*(usr_time+a_p(4)-t(3));
    sqrt((x(4)-a_p(1))^2+(y(4)-a_p(2))^2+(z(4)-a_p(3))^2)-c*(usr_time+a_p(4)-t(4))];
end