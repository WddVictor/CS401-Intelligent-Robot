close all;
clear all;

rosshutdown;
rosinit;
sim = ExampleHelperRobotSimulator('simpleMap');
setRobotPose(sim,[2 3 -pi/2]);

enableROSInterface(sim,true);

sim.LaserSensor.NumReadings = 50;

scanSub=rossubscriber('scan');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');

tftree = rostf;
pause(1);

path = [2,3;3.25 6.25;2 11;6 7;11 11;8 6;10 5;7 3;11 1.5];
plot(path(:,1),path(:,2),'k--d');

controller = robotics.PurePursuit('Waypoints',path);
controller.DesiredLinearVelocity=0.4;

controlRate=robotics.Rate(10);

goalRadius = 0.1;
robotCurrentLocation=path(1,:);
robotGoal=path(end,:);
distanceToGoal = norm(robotCurrentLocation-robotGoal);

map = robotics.OccupancyGrid(14,13,20);

figureHandle=figure('Name','Map');
axesHandle = axes('Parent',figureHandle);
mapHandle = show(map,'Parent',axesHandle);
title(axesHandle,'OccupancyGrid:Update 0');

updateCounter = 1;

while(distanceToGoal > goalRadius)
    
    scanMsg = receive(scanSub);
    
    pose = getTransform(tftree,'map','robot_base',scanMsg.Header.Stamp,'Timeout',2);
    
    position = [pose.Transform.Translation.X,pose.Transform.Translation.Y];
    orientation = quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
        pose.Transform.Rotation.Y,pose.Transform.Rotation.Z],'ZYX');
    
    robotPose = [position,orientation(1)];
    
    scan = lidarScan(scanMsg);
    ranges = scan.Ranges;
    ranges(isnan(ranges)) = sim.LaserSensor.MaxRange;
    modScan = lidarScan(ranges,scan.Angles);
    
    insertRay(map,robotPose,modScan,sim.LaserSensor.MaxRange);
    
    [v, w] = controller(robotPose);
    velMsg.Linear.X=v;
    velMsg.Angular.Z=w;
    send(velPub,velMsg);
    
    if ~mod(updateCounter,50)
        mapHandle.CData = occupancyMatrix(map);
        title(axesHandle,['OccupancyGrid: Update' num2str(updateCounter)]);
    end
    
    updateCounter = updateCounter+1;
    distanceToGoal = norm(robotPose(1:2)-robotGoal);
    
    waitfor(controlRate);
    
end

show(map,'Parent',axesHandle);
title(axesHandle,'OccupancyGride: Final Map');


function myInsertRay(obj, varargin)
gridSize = obj.GridSize;
res = obj.Resolution;
loc = obj.GridLocationInWorld;

[startPt, endPt, inverseModelLogodds, rangeIsMax] = obj.parseInsertRayInputs(varargin{:});

for i = 1:size(endPt, 1)
    [endPts, middlePts] = robotics.algs.internal.raycastCells(startPt, endPt(i,:), ...
        gridSize(1), gridSize(2), res, loc);
    
    if ~isempty(middlePts)
        mIndex = sub2ind(gridSize, middlePts(:,1), middlePts(:,2));
        updateValuesMiss = obj.Logodds(mIndex) + inverseModelLogodds(1);
        updateValuesMiss(updateValuesMiss < obj.ProbSatIntLogodds(1)) = obj.ProbSatIntLogodds(1);
        obj.Logodds(mIndex) = updateValuesMiss;
    end
    
    if ~isempty(rangeIsMax) && rangeIsMax(i)
        continue;
    end
    if ~isempty(endPts)
        eIndex = sub2ind(gridSize, endPts(:,1), endPts(:,2));
        updateValuesHit = obj.Logodds(eIndex) + inverseModelLogodds(2);
        updateValuesHit(updateValuesHit > obj.ProbSatIntLogodds(2)) = obj.ProbSatIntLogodds(2);
        obj.Logodds(eIndex) = updateValuesHit;
    end
end
end