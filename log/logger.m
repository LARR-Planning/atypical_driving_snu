%% log corridor information 


horizon_global = 5;
accel_lim = [-1.0 1.0];
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
dd = 20;
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
    cur_idx = find(data_mpc(:,1) == cur_time);
    mpc_start = cur_idx(1);
    
    subplot(4,1,1)
    title('x')

    plot(data_mpc(mpc_start,2:end),data_mpc(mpc_start+1,2:end),'r')
    hold off
    subplot(4,1,2)
    title('y')

    plot(data_mpc(mpc_start,2:end),data_mpc(mpc_start+2,2:end),'g')
    hold off
    
    cur_line = corr_end + 1;
    
    subplot(4,1,3)
    hold on
    yline(accel_lim(1),'r--')
    yline(accel_lim(2),'r--')
    title('accel decel')
    plot(data_mpc(mpc_start,2:end),data_mpc(mpc_start+3,2:end),'b')
    plot(data_mpc(mpc_start,2),data_mpc(mpc_start+3,2),'ko')

    
    
    
    
    subplot(4,1,4)
    hold on
    yline((angluar_lim(1)),'r--')
    yline(angluar_lim(2),'r--')
    title('angular')   
    plot(data_mpc(mpc_start,2:end),data_mpc(mpc_start+4,2:end),'c')
    plot(data_mpc(mpc_start,2),data_mpc(mpc_start+4,2),'ko')
    pause(1)

    if cur_line  < size(data_corridor,1)
    clf
    end
end












