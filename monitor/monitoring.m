topic = '/atypical_planning_test/monitor/status';
bag = rosbag('originalArisimScenario.bag');
bSel = select(bag,'Topic',topic);
msgStruct = readMessages(bSel,'DataFormat','struct');

%% data logging 
distToStaticObstacle = [];
distToDynamicObstacles = [];
compTime = [];
for n = 1:length(msgStruct)
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
hDynamic = plot (distToDynamicObstacles ,'r-' );
hAvoid = yline(1,'k:','LineWidth',2);
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
