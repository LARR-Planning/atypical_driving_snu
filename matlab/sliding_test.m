%% 1. Offline
clear;
close all;
clc;

%% Set parameters at here!!
bagfile_path = '../worlds/occupancy_inflated_seq.bag'; 
nav_msg_topic_name = '/costmap_node/costmap/costmap';
car_width = 2;
grid_resolution = 0.5;
threshold = 0.5;
max_steering_angle = pi/4;
max_smoothing_iteration = 5;
smoothing_margin = 0.3;


% current_state.position = [0 + 25 0 + 15];
% current_state.velocity = 0;
% current_state.theta = pi/2;
% goal_state.position = [50 + 25 70 + 15];

lanePath = [1 0 ; 
            1 71; 
            50 71;]; % lanePath = [lanePoint; lanePoint; lanePoint; ...]
laneWidth = [8;
                   12];

%% Lane path preprocessing
laneAngle = zeros([size(lanePath, 1) - 1, 1]);
for i = 1:size(lanePath)-1
    direction = lanePath(i+1,:) - lanePath(i,:);
    laneAngle(i) = atan2(direction(2), direction(1));
end
% get laneGridPoint before transformation
laneGridPoint_ = [];
laneGridAngle = [];
laneGridWidth = [];
for i = 1:size(lanePath)-1
    lanePath_length = norm(lanePath(i+1,:) - lanePath(i,:));
    current_length = 0;
    while current_length < lanePath_length
        alpha = current_length / lanePath_length;
        laneGridPoint_ = [laneGridPoint_; lanePath(i+1,:) * alpha + lanePath(i,:) * (1-alpha)];
        laneGridAngle = [laneGridAngle; laneAngle(i)];
        laneGridWidth = [laneGridWidth; laneWidth(i)];
        current_length = current_length + grid_resolution;
    end
end

%% 2. Realtime Works
%% Read map
bag = rosbag(bagfile_path);
bSel = select(bag,'Topic',nav_msg_topic_name);
msgStructs = readMessages(bSel,'DataFormat','struct');

figure(1)
set(gcf,'Position',[100 100 1000 1000])
for i_msg = 1 : size(msgStructs,1)
msgMap = msgStructs{i_msg};
msg = rosmessage('nav_msgs/OccupancyGrid');
msg.Info.Height = msgMap.Info.Height;
msg.Info.Width = msgMap.Info.Width;
msg.Info.Resolution = msgMap.Info.Resolution;
msg.Data = msgMap.Data;
map = readBinaryOccupancyGrid(msg);

% %% Find closest point to lanePath
% [p_lane_start, lane_index_start] = getCloestPointToLanePath(current_state.position, lanePath, 1);
% [p_lane_goal, lane_index_goal] = getCloestPointToLanePath(goal_state.position, lanePath, 1);


%% update initialLane
translation = [msgMap.Info.Origin.Position.X msgMap.Info.Origin.Position.Y]
laneGridPoint = laneGridPoint_ - translation;


%% Grid generation
%for laneGridPoint check side point and get middle points
i_tree = 1;
laneTree = [];
% leftGrid = [];
% rightGrid = [];


for i = 1:size(laneGridPoint)
    col_size = 2 * (2 * floor(laneGridWidth(i)/2/grid_resolution) + 1);
    side_point = zeros(col_size/2, 2);
    
    left_angle = laneGridAngle(i) + pi/2;
    right_angle = laneGridAngle(i) - pi/2; 
        
    side_point(floor(laneGridWidth(i)/2/grid_resolution)+1,:) = laneGridPoint(i,:);
    for j = 1:floor(laneGridWidth(i)/2/grid_resolution)
        left_point = laneGridPoint(i,:) + j * grid_resolution * [cos(left_angle) sin(left_angle)];
        right_point = laneGridPoint(i,:) + j * grid_resolution * [cos(right_angle) sin(right_angle)];
        
        side_point(floor(laneGridWidth(i)/2/grid_resolution)+1+j,:) = left_point;
        side_point(floor(laneGridWidth(i)/2/grid_resolution)+1-j,:) = right_point;
    end
    
    start_idx = 0;
    for k = 1:size(side_point,1)
        if checkOccupancy(map,side_point(k,:)) < threshold
            if start_idx == 0
                start_idx = k;
            end
            if k == size(side_point,1)
                midPoint = (side_point(k,:) + side_point(start_idx,:))/2;
                parents = findParents(map, laneTree, i, midPoint);
                laneTree(i_tree,:).id = i;
                laneTree(i_tree,:).leftPoint = side_point(k,:);
                laneTree(i_tree,:).midPoint = midPoint;
                laneTree(i_tree,:).rightPoint = side_point(start_idx,:);
                laneTree(i_tree,:).parents = parents;
                i_tree = i_tree + 1;
            end
        elseif start_idx > 0
            midPoint = (side_point(k-1,:) + side_point(start_idx,:))/2;
            parents = findParents(map, laneTree, i, midPoint);
            laneTree(i_tree,:).id = i;
            laneTree(i_tree,:).leftPoint = side_point(k-1,:);
            laneTree(i_tree,:).midPoint = midPoint;
            laneTree(i_tree,:).rightPoint = side_point(start_idx,:);
            laneTree(i_tree,:).parents = parents;
            i_tree = i_tree + 1;
            start_idx = 0;
        else
            start_idx = 0;
        end
    end
end

%% Find proper collision-free midPoint
i_tree = size(laneTree, 1);
tail = laneTreeDFS(laneTree, i_tree);
midPoints = [];

if tail == -1
    hold on
    show(map)
    
    for i_tree = 1:size(laneTree, 1)    
        midPoints(i_tree, :) = laneTree(i_tree).midPoint;
        parents = laneTree(i_tree).parents;
        if i_tree ~= 1
            for i_parents = 1:size(parents,2)
                lineSegment = [midPoints(i_tree, :); laneTree(laneTree(i_tree).parents(i_parents)).midPoint];
                plot(lineSegment(:,1), lineSegment(:,2));
            end
        end
    end
    scatter(midPoints(:,1), midPoints(:,2), '.')
    
    
    hold off
    pause(0.01)
end

midPoints = zeros(size(tail, 2), 2);
for i_tail = 1:size(tail, 2)
    midPoints(i_tail, :) = laneTree(tail(i_tail)).midPoint;
end

leftPoints = zeros(size(tail, 2), 2);
for i_tail = 1:size(tail, 2)
    leftPoints(i_tail, :) = laneTree(tail(i_tail)).leftPoint;
end

rightPoints = zeros(size(tail, 2), 2);
for i_tail = 1:size(tail, 2)
    rightPoints(i_tail, :) = laneTree(tail(i_tail)).rightPoint;
end


%% Find midGridAngle
midAngle = zeros(size(midPoints,1)-1, 1);
for i = 1:size(midAngle, 1)
    delta_angle = midPoints(i+1,:) - midPoints(i,:);
    angle = atan2(delta_angle(2), delta_angle(1));
    if(angle < 0)
        angle = angle + 2 * pi;
    end
    midAngle(i) = angle;
end

%% Smoothing using linear interpolation TODO: valid smoothing check
i = 1;
while i < size(midAngle, 1)
%     if(angDiff(midAngle(i+1), midAngle(i)) > max_steering_angle)
%         idx_start = i;
%         idx_end = min(i+3, size(midPoints,1));
%         delta = idx_end - idx_start;
%         for j = 1:delta-1
%             alpha = j/delta;
%             smoothingPoint = (1-alpha) * midPoints(idx_start,:) + alpha * midPoints(idx_end,:);
%             midPoints(idx_start+j,:) = smoothingPoint;
%         end
%         for j = 0:delta-1
%             delta_angle = midPoints(idx_start+j+1,:) - midPoints(idx_start+j,:);
%             angle = atan2(delta_angle(2), delta_angle(1));
%             if(angle < 0)
%                 angle = angle + 2 * pi;
%             end
%             midAngle(idx_start+j) = angle;
%         end
%         i = max(i-1 , 1);
%     else
%         i = i + 1;    
%     end
    for i_smooth = 0:max_smoothing_iteration-1
        idx_start = max(i - i_smooth, 1);
        idx_end = min(i+3+i_smooth, size(midPoints,1));
        delta = idx_end - idx_start;
        smoothingPoint = zeros(delta-1, 2);
        is_smoothing_valid = true;
        
        for j = 1:delta-1
            alpha = j/delta;
            smoothingPoint(j,:) = (1-alpha) * midPoints(idx_start,:) + alpha * midPoints(idx_end,:);
            leftMargin = leftPoints(idx_start+j,:) - smoothingPoint(j,:);
            rightMargin = rightPoints(idx_start+j,:) - smoothingPoint(j,:);
            if dot(leftMargin, rightMargin) >= 0 || norm(leftMargin) < smoothing_margin || norm(rightMargin) < smoothing_margin
                % collision occured!
                is_smoothing_valid = false;
            end
        end
        
        if is_smoothing_valid
            for j = 1:delta-1
                midPoints(idx_start+j,:) = smoothingPoint(j,:);
            end
        else
            break;
        end
    end

    i = i + 1;    
end

%% Plot map and grid
hold on
show(map)
scatter(midPoints(:,1), midPoints(:,2), '.', 'b')
scatter(leftPoints(:,1), leftPoints(:,2), '.', 'r')
scatter(rightPoints(:,1), rightPoints(:,2), '.', 'g')
hold off

% figure
% plot(midGridAngle)

pause(0.01)
end


