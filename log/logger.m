%% log 0622 (use this)

data_input = load('log_input.txt');
data_state = load('log_state.txt');
data_mpc = load('log_mpc.txt');


figure(1)

hold on
plot(data_state(:,2),data_state(:,3),'k-')
% comet(data_state(:,2),data_state(:,3))

theta = data_state(:,4);
xaxis = [cos(theta) sin(theta)];
dd = 50;
quiver(data_state(1:dd:end,2),data_state(1:dd:end,3),xaxis(1:dd:end,1),xaxis(1:dd:end,2))
xlabel('x')
ylabel('y')

figure(2)
subplot(2,1,1)
gg = plot(data_input(:,1),data_input(:,2),'b-','LineWidth',4);
gg.Color(4) = 0.2;
hold on
title('accel history')
subplot(2,1,2)
gg = plot(data_input(:,1),data_input(:,3),'c-','LineWidth',4);
gg.Color(4) = 0.2;
hold on
title('steering history')
%% mpc RHP 
figure(3)
Ndata = size(data_mpc,1)/5;

for n = 1:Ndata
    ts = data_mpc((n-1)*5+1,2:end);
    xs = data_mpc((n-1)*5+2,2:end);
    ys = data_mpc((n-1)*5+3,2:end);
    as = data_mpc((n-1)*5+4,2:end);
    dels = data_mpc((n-1)*5+5,2:end);
    amax = ones(1,length(ts))*1;
    amin = -ones(1,length(ts))*3;
    delmax = ones(1,length(ts))*30;
    delmin = -ones(1,length(ts))*30;

    subplot(4,2,1)
    plot(ts,xs)
    title('x[m]')

    subplot(4,2,3)
    plot(ts,ys)
    title('y[m]')

    subplot(4,2,5)
    plot(ts,as,ts,amax,'r--',ts,amin,'r--')
    title('a[m/s^2]')
    
    subplot(4,2,7)
    plot(ts,dels*180/pi,ts,delmax,'r--',ts,delmin,'r--')
    title('angle[degree]')
    
    subplot(4,2,[2 4 6 8])
    plot(xs,ys,'r-',data_state(:,2),data_state(:,3),'k-')
    axis equal
    title(strcat('id = ',num2str(n)))
    pause(1e-2)    
end


%% future offset mpc vs current 
figure(4)
Ndata = size(data_mpc,1)/5;
futureStride = 10;
tMPC = [];
xMPC = [];
yMPC = [];
for n = 1:Ndata
    t = data_mpc((n-1)*5+1,2+futureStride);
    x = data_mpc((n-1)*5+2,2+futureStride);
    y = data_mpc((n-1)*5+3,2+futureStride);    
    tMPC = [tMPC t];
    xMPC = [xMPC x];
    yMPC = [yMPC y];
end
subplot(2,1,1)
hold on
plot(tMPC,xMPC,'r-')
plot(data_state(:,1),data_state(:,2),'k-')
subplot(2,1,2)
hold on 
plot(tMPC,yMPC,'r-')
plot(data_state(:,1),data_state(:,3),'k-')



%% log corridor information 
horizon_global = 5;
accel_lim = [-3.0 1.0];
angluar_lim = [-0.52 0.52];

data_corridor = load('log_corridor.txt');
data_mpc = load('log_mpc.txt');
data_input = load('log_input.txt');
data_state = load('log_state.txt');

cur_line = 1;
fig_corridor = 1;
fig_state = 2;

figure(fig_state)
hold on
plot(data_state(:,2),data_state(:,3),'k-')
theta = data_state(:,4);
xaxis = [cos(theta) sin(theta)];
dd = 50;
quiver(data_state(1:dd:end,2),data_state(1:dd:end,3),xaxis(1:dd:end,1),xaxis(1:dd:end,2))
xlabel('x')
ylabel('y')

figure(fig_corridor)
subplot(4,1,3)
gg = plot(data_input(:,1),data_input(:,2),'b-','LineWidth',4);
gg.Color(4) = 0.2;
hold on
subplot(4,1,4)
gg = plot(data_input(:,1),data_input(:,3),'c-','LineWidth',4);
gg.Color(4) = 0.2;
hold on

while cur_line <= size(data_corridor,1)
    figure(fig_corridor)
    subplot(4,1,3)
    gg = plot(data_input(:,1),data_input(:,2),'b-','LineWidth',4);
    gg.Color(4) = 0.2;
    hold on
    subplot(4,1,4)
    gg = plot(data_input(:,1),data_input(:,3),'c-','LineWidth',4);
    gg.Color(4) = 0.2;
    hold on

    % Corridor 
    corr_start = cur_line; 
    cur_idx = find(data_corridor(:,1) == data_corridor(corr_start,1));
    corr_end = cur_idx(end);
    
    cur_time = data_corridor(corr_start,1);
    t1 = cur_time;
    tmax = cur_time+horizon_global;
    
    figure(fig_corridor)
    
    for idx = corr_start : corr_end
       t2 = min(t1 + data_corridor(idx,3),tmax);
       subplot(4,1,1)
       hold on
       plot([t1 min(t2,tmax)],[data_corridor(idx,4) data_corridor(idx,4)],'k-');
       plot([t1 min(t2,tmax)],[data_corridor(idx,5) data_corridor(idx,5)],'k-');
    
       subplot(4,1,2)
       hold on
       plot([t1 min(t2,tmax)],[data_corridor(idx,6) data_corridor(idx,6)],'k-');
       plot([t1 min(t2,tmax)],[data_corridor(idx,7) data_corridor(idx,7)],'k-');
       
       t1 = t2; 
    end
        
    % Corridor 
    figure(fig_corridor)
    cur_idx = find(abs(data_mpc(:,1) - cur_time)<0.001);
    mpc_start = cur_idx(1);
    
    subplot(4,1,1)
    title('x')

    plot(data_mpc(mpc_start,2:end),data_mpc(mpc_start+1,2:end),'r')
    hold off
    subplot(4,1,2)
    title('y')

    plot(data_mpc(mpc_start,2:end),data_mpc(mpc_start+2,2:end),'g')
    hold off
    
    cur_line = corr_end + 10;
    
    subplot(4,1,3)
    hold on
    yline(accel_lim(1),'r--')
    yline(accel_lim(2),'r--')
    title('accel decel')
    plot(data_mpc(mpc_start,2:end),data_mpc(mpc_start+3,2:end),'b','LineWidth',5)
    plot(data_mpc(mpc_start,2),data_mpc(mpc_start+3,2),'ko')

    
    
    
    
    subplot(4,1,4)
    hold on
    yline((angluar_lim(1)),'r--')
    yline(angluar_lim(2),'r--')
    title('angular')   
    plot(data_mpc(mpc_start,2:end),data_mpc(mpc_start+4,2:end),'c','LineWidth',5)
    plot(data_mpc(mpc_start,2),data_mpc(mpc_start+4,2),'ko')
    
    figure(fig_state)
    hold on
    plot(data_state(:,2),data_state(:,3),'k-')
    theta = data_state(:,4);
    xaxis = [cos(theta) sin(theta)];
    dd = 20;
    quiver(data_state(1:dd:end,2),data_state(1:dd:end,3),xaxis(1:dd:end,1),xaxis(1:dd:end,2),'r')
    xlabel('x')
    ylabel('y')
    
    [~,cur_idx] = min(abs(data_state(:,1) - cur_time));
    xCur = data_state(cur_idx,2);
    yCur = data_state(cur_idx,3);
    plot(xCur,yCur,'ko','MarkerSize',10);
    
    
    pause(1e-1)

    if cur_line  < size(data_corridor,1)
        figure(fig_corridor)

        clf
    end
end



%% 

bag = rosbag('../worlds/keti_dataset1.bag');
bSel = select(bag,'Topic','/current_speed');
msgStructs = readMessages(bSel,'DataFormat','struct');
speed = [];
for i = 1:length(msgStructs)
   speed = [speed msgStructs{i}];
end



%% Prediction 

data_observation= load('predictor_saved/observation_35.txt');
figure(1)
subplot(4,2,1)
plot(data_observation(:,1),'r-');
title('x')
subplot(4,2,3)
plot(data_observation(:,2),'g-');
title('y')
subplot(4,2,5)
plot(data_observation(:,7),'k-');
title('dimX')
subplot(4,2,7)
plot(data_observation(:,8),'k-');
title('dimY')

subplot(4,2,2)
plot(data_observation(:,3),'r-');
title('qx')
subplot(4,2,4)
plot(data_observation(:,4),'g-');
title('qy')
subplot(4,2,6)
plot(data_observation(:,5),'b-');
title('qz')
subplot(4,2,8)
plot(data_observation(:,6),'k-');
title('qw')
sgtitle('id = 35')



%% logger for pitch angle 


bag = rosbag('imu_pitch_log_nice_but_nominal_too_fast.bag');
bSel = select(bag,'Topic','/atypical_planning_test/imu_pitcing');
msgStructs = readMessages(bSel,'DataFormat','struct');

pitchSet = [];
for n = 1:length(msgStructs)
    pitchSet = [pitchSet msgStructs{n}.Data] ;
end

plot(pitchSet*180/pi)

%% 
bag = rosbag('/home/jbs/2020-11-14-21-17-09.bag');
bSel = select(bag,'Topic','/atypical_planning_test/cur_pose');

bSel2 = select(bag,'Topic','/current_pose');


msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs2 = readMessages(bSel2,'DataFormat','struct');

% Ours
xs = [];
ys = [];
zs = [];
ts = [];
% KETI 
xs_keti = [];
ys_keti = [];
zs_keti = [];
ts_keti = [];

for n = 1:length(msgStructs)
    position = msgStructs{n}.Pose.Position;
    xs = [xs position.X];
    ys = [ys position.Y];
    zs = [zs position.Z];   
    ts= [ts double(msgStructs{n}.Header.Stamp.Sec) + double(msgStructs{n}.Header.Stamp.Nsec)*1e-9 ];
end


for n = 1:length(msgStructs2)
    position = msgStructs2{n}.Pose.Position;
    xs_keti = [xs_keti position.X];
    ys_keti = [ys_keti position.Y];
    zs_keti = [zs_keti position.Z];   
end


figure(2)
subplot(2,1,1)
hold on
% plot(ts,xs)
% plot(ts,ys)
% plot(ts,zs)
ts  = ts - ts(1);
markerSize = 2;
    plot(ts,xs,'o','MarkerSize',markerSize);
    plot(ts,ys,'o','MarkerSize',markerSize);
    plot(ts,zs,'o','MarkerSize',markerSize);
set(gca,'XLim',[0 10])

subplot(2,1,2)
hold on
    plot(xs_keti(1:4:end)-xs_keti(1),'o','MarkerSize',markerSize);
    plot(ys_keti(1:4:end)-ys_keti(1),'o','MarkerSize',markerSize);
    plot(zs_keti(1:4:end)-zs_keti(1),'o','MarkerSize',markerSize);
set(gca,'XLim',[0 500])
set(gca,'XLim',[0 500])

%% 


