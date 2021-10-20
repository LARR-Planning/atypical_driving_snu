topic = '/atypical_planning_test/monitor/status';
bag = rosbag('track1.bag');
bSel = select(bag,'Topic',topic);
msgStruct = readMessages(bSel,'DataFormat','struct');

%% data logging 
distToStaticObstacle = [];
distToDynamicObstacles = [];
compTime = [];
for n = 40:length(msgStruct)
   distToStaticObstacle = [distToStaticObstacle ...
       msgStruct{n}.DistStaticObstacle];
   distToDynamicObstacles = [distToDynamicObstacles ...
       msgStruct{n}.DistDynamicObstacles];
   compTime = [compTime msgStruct{n}.CompTimeMs ];
end

figure(1)
clf
set(gcf,'Position',[961 1 960 995])
subplot(2,1,1)
hold on 
title('Distance to obstacles [m]')
hStatic = plot (distToStaticObstacle ,'b-' );
hStaticAvg = yline(mean(distToStaticObstacle),'b--');
hDynamic = plot (rmoutliers(distToDynamicObstacles) ,'r-' ); 
hAvoid = yline(1.4,'r:','LineWidth',2);
hProximityCrit = yline(8,'b:','LineWidth',2);
text(100,1.9,'1.4 m','FontSize',15)
text(100,9,'8 m','FontSize',15)

legend([hStatic hStaticAvg hDynamic ],...
    {'StaticProximity>0 m ','StaticProximityAvg < 8 m','DistToDynamic > 1.0+0.4 m'},...
    'FontSize',14,'Location','northwest')
xlabel('data points')
set(gca,'FontSize',15)

subplot(2,1,2)
hold on
title('Computation time [ms]')
hCompTime = plot (compTime ,'k-' );
yline(50,'k:','LineWidth',2)
hMean = yline(mean(compTime),'k--','LineWidth',1.5);
xlabel('data points')
legend(hMean,'avg.','FontSize',14,'Location','northwest')
set(gca,'YLim',[0 60])
set(gca,'FontSize',15)


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



