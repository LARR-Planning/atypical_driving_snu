%% 1. Offline
clear;
close all;
clc;

%% Set parameters at here!!
bagfile_path = '../worlds/occupancy_grid.bag'; 
nav_msg_topic_name = '/projected_map';
car_width = 2;
grid_resolution = 0.5;
road_width = 12;
threshold = 0.5;
n_poly = 10;
n_smoothing_points = 5;
max_steering_angle = pi/4;

% [80,20]-> [20,20] -> [20,60]
current_state.position = [80,20];
current_state.velocity = 0;
current_state.theta = pi;
goal_state.position = [20,60];

lanePath = [90 22; 
            20 22; 
            20 60;]; % lanePath = [lanePoint; lanePoint; lanePoint; ...]

%% Lane path preprocessing
laneAngle = zeros([size(lanePath, 1) - 1, 1]);
for i = 1:size(lanePath)-1
    direction = lanePath(i+1,:) - lanePath(i,:);
    laneAngle(i) = atan2(direction(2), direction(1));
end
% get laneGridPoint
laneGridPoint = [];
laneGridAngle = [];
for i = 1:size(lanePath)-1
    lanePath_length = norm(lanePath(i+1,:) - lanePath(i,:));
    current_length = 0;
    while current_length < lanePath_length
        alpha = current_length / lanePath_length;
        laneGridPoint = [laneGridPoint; lanePath(i+1,:) * alpha + lanePath(i,:) * (1-alpha)];
        laneGridAngle = [laneGridAngle; laneAngle(i)];
        current_length = current_length + grid_resolution;
    end
end

%% 2. Realtime Works
%% Read map
bag = rosbag(bagfile_path);
bSel = select(bag,'Topic',nav_msg_topic_name);
msgStructs = readMessages(bSel,'DataFormat','struct');
msgMap = msgStructs{1};
msg = rosmessage('nav_msgs/OccupancyGrid');
msg.Info.Height = msgMap.Info.Height;
msg.Info.Width = msgMap.Info.Width;
msg.Info.Resolution = msgMap.Info.Resolution;
msg.Data = msgMap.Data;
map = readBinaryOccupancyGrid(msg);

%% Inflate map
inflate(map, car_width/2);

%% Find closest point to lanePath
 [p_lane_start, lane_index_start] = getCloestPointToLanePath(current_state.position, lanePath, 1);
 [p_lane_goal, lane_index_goal] = getCloestPointToLanePath(goal_state.position, lanePath, 1);
 
%% Grid generation
%for laneGridPoint check side point and get middle points
row_size = size(laneGridPoint, 1);
col_size = 2 * (2 * floor(road_width/2/grid_resolution) + 1);
grid = zeros(row_size, col_size);
midGridPoint = zeros(row_size, 2);
leftGridPoint = zeros(row_size, 2);
rightGridPoint = zeros(row_size, 2);
side_point = zeros(col_size/2, 2);

for i = 1:size(laneGridPoint)
    left_angle = laneGridAngle(i) + pi/2;
    right_angle = laneGridAngle(i) - pi/2; 
        
    grid(i, col_size/2:col_size/2+1) = laneGridPoint(i,:);
    side_point(floor(road_width/2/grid_resolution)+1,:) = laneGridPoint(i,:);
    for j = 1:floor(road_width/2/grid_resolution)
        left_point = laneGridPoint(i,:) + j * grid_resolution * [cos(left_angle) sin(left_angle)];
        right_point = laneGridPoint(i,:) + j * grid_resolution * [cos(right_angle) sin(right_angle)];
        
        side_point(floor(road_width/2/grid_resolution)+1+j,:) = left_point;
        side_point(floor(road_width/2/grid_resolution)+1-j,:) = right_point;
        
        grid(i, col_size/2-2*j:col_size/2+1-2*j) = left_point;
        grid(i, col_size/2+2*j:col_size/2+1+2*j) = right_point;
    end
    
    count = 0;
    start_idx = 0;
    max_count = 0;
    for k = 1:size(side_point,1)
        if checkOccupancy(map,side_point(k,:)) < threshold
            if start_idx == 0
                start_idx = k;
            end
            count = count + 1;
            if k == size(side_point,1) && count > max_count
                leftGridPoint(i,:) = side_point(k,:);
                midGridPoint(i,:) = (side_point(k,:) + side_point(start_idx,:))/2;
                rightGridPoint(i,:) = side_point(start_idx,:);                
                max_count = count;
            end
        elseif start_idx > 0 && count > max_count
            leftGridPoint(i,:) = side_point(k-1,:);
            midGridPoint(i,:) = (side_point(k-1,:) + side_point(start_idx,:))/2;
            rightGridPoint(i,:) = side_point(start_idx,:);
            max_count = count;
            start_idx = 0;
            count = 0;
        else
            start_idx = 0;
            count = 0;
        end
    end
    
    if max_count == 0
        debug = 1;
    end
end


%% Find midGridAngle
midGridAngle = zeros(size(midGridPoint,1)-1, 1);
for i = 1:size(midGridAngle, 1)
    delta_angle = midGridPoint(i+1,:) - midGridPoint(i,:);
    angle = atan2(delta_angle(2), delta_angle(1));
    if(angle < 0)
        angle = angle + 2 * pi;
    end
    midGridAngle(i) = angle;
end

%% Polynomial fitting
poly_fitting = polyfit(midGridPoint(:,1), midGridPoint(:,2), n_poly);
x_plot = linspace(20, 90);
y_plot = polyval(poly_fitting, x_plot);

%% Linear interpolation
i = 1;
while i < size(midGridAngle, 1)
    if(abs(midGridAngle(i+1) - midGridAngle(i)) > max_steering_angle)
        idx_start = i;
        idx_end = min(i+3, size(midGridPoint,1));
        delta = idx_end - idx_start;
        for j = 1:delta-1
            alpha = j/delta;
            midGridPoint(idx_start+j,:) = (1-alpha) * midGridPoint(idx_start,:) + alpha * midGridPoint(idx_end,:);
        end
        for j = 0:delta-1
            delta_angle = midGridPoint(idx_start+j+1,:) - midGridPoint(idx_start+j,:);
            angle = atan2(delta_angle(2), delta_angle(1));
            if(angle < 0)
                angle = angle + 2 * pi;
            end
            midGridAngle(idx_start+j) = angle;
        end
        
        i = max(i-1 , 1);
    else
        i = i + 1;    
    end
    
end

%% Plot map and grid
hold on
show(map)
% for j = 1:col_size/2
%     scatter(grid(:,2*j-1), grid(:,2*j), '.')
% end
scatter(midGridPoint(:,1), midGridPoint(:,2), '.')
scatter(leftGridPoint(:,1), leftGridPoint(:,2), '.')
scatter(rightGridPoint(:,1), rightGridPoint(:,2), '.')
% plot(x_plot, y_plot)
hold off

figure
plot(midGridAngle)





