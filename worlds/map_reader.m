bag = rosbag('occupancy_grid.bag');
bSel = select(bag,'Topic','/projected_map');
msgStructs = readMessages(bSel,'DataFormat','struct');
msgMap = msgStructs{1};

%% 
msg = rosmessage('nav_msgs/OccupancyGrid');
msg.Info.Height = msgMap.Info.Height;
msg.Info.Width = msgMap.Info.Width;
msg.Info.Resolution = msgMap.Info.Resolution;
msg.Data = msgMap.Data;
map = readBinaryOccupancyGrid(msg);
show(map)

