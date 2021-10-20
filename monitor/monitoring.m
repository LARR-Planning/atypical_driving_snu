topic = '/atypical_planning_test/monitor/status';
bag = rosbag('_2021-10-20-09-41-14.bag');
bSel = select(bag,'Topic',topic);
msgStruct = readMessages(bSel,'DataFormat','struct');

%% data logging 
distToStaticObstacle = [];
distToDynamicObstacles = [];
compTime = [];
for n = 10:length(msgStruct)
   distToStaticObstacle = [distToStaticObstacle ...
       msgStruct{n}.DistStaticObstacle];
   distToDynamicObstacles = [distToDynamicObstacles ...
       msgStruct{n}.DistDynamicObstacles];
   compTime = [compTime msgStruct{n}.CompTimeMs ];
end

figure(1)
subplot(3,1,1)
hold on 
title('Distance to obstacles [m]')
hStatic = plot (distToStaticObstacle ,'b-' );
hStaticAvg = yline(mean(distToStaticObstacle),'b--');
hDynamic = plot (rmoutliers(distToDynamicObstacles) ,'r-' ); % to remove jerky obstacle state 
hAvoid = yline(1.2,'r:','LineWidth',2);
hProximityCrit = yline(8,'k:','LineWidth',2);

legend([hStatic hStaticAvg hDynamic ],...
    {'StaticProximity','StaticProximityAvg','DistToDynamic'},...
    'FontSize',14)

subplot(3,1,2)
hold on
title('Computation time [ms]')
hCompTime = plot (compTime ,'b-' );
yline(50,'k:','LineWidth',2)
yline(mean(compTime),'b--','LineWidth',1.5)

%% Dynamic objects sub monitoring 
topic = '/detected_objects';
bag = rosbag('../worlds/track1.bag');
bSel = select(bag,'Topic',topic);
msgStruct = readMessages(bSel,'DataFormat','struct');

M = length(msgStruct{n}.Objects);
for m = 1: 1
    dynamicPos{m}.X = []; 
    dynamicPos{m}.Y = []; 
    for n= 1 : length(msgStruct)
        x = msgStruct{n}.Objects(m).Odom.Pose.Pose.Position.X;
        y = msgStruct{n}.Objects(m).Odom.Pose.Pose.Position.Y;
        dynamicPos{m}.X = [dynamicPos{m}.X x];
        dynamicPos{m}.Y = [dynamicPos{m}.Y y];    
    end
end



