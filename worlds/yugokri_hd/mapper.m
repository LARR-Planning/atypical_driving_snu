pcdFileDir ='yugokri_hd.pcd'; % This was generated w.r.t SNU frame (start = origin)
laneFileDir = 'interpolated_yugokri_path1.csv'; % w.r.t global frame 
originPoseDir = 'world_to_init'; % copy and paste 

%% Load pcd, transform, and lane 

% PCD downsampling
pcdFileRaw = pcread(pcdFileDir);
pcdFile = pcdownsample(pcdFileRaw,'gridAverage',1);
pcdPoints = pcdFile.Location;
pcdObj = pointCloud(pcdPoints);

% Read lane from here 
lane = table2array(readtable('interpolated_yugokri_path1.csv'));
zOffSet = 0.4;
lane3d = [lane zOffSet*ones(size(lane,1),1)];

%% Draw HD map and lane  
pcshow(pcdObj,'MarkerSize',2);
colormap(gray)
xlabel('x')
ylabel('y')
view([0 90])
hold on 
plot3(lane3d(:,1),lane3d(:,2),lane3d(:,3),'w','LineWidth',2)
plot3(trans(1),trans(2),trans(3),'wo','MarkerFaceColor','w')

% Sectioning - use Display Tip along white curve from above
% .... do fucking things from mouse clicking




%% Sectioning knots and bind index of lane 
data = load('section_division_pnts.mat');
knots = struct2cell(data); nKnot = length(knots);
knotsAug = knots; % to close loop
knotsAug(nKnot) = knots(1);
searchStart = 1;

for n = 1:nKnot
    pnt = knotsAug{n}.Position(1:2);
    ind = getClosestIndex(pnt,lane,searchStart);
    knotWithIndex{n}.pnt = pnt;
    knotWithIndex{n}.ind = ind; 
end

%% Visualize knots 
for n = 1:nKnot
    pnt = knotWithIndex{n}.pnt;
    ind = knotWithIndex{n}.ind;    
    plot3(pnt(1),pnt(2),zOffSet,'ro','MarkerFaceColor','r')
end


%% Subfunctions

function indexClosest = getClosestIndex(pnt,pntArray,searchIndex)
    % pntArray = [Narray x dim]
    % pnt = [1 x dim]
    % searchIndex = Lookup from this index 
    
    indexClosest = searchIndex;
    distClosest = norm(pnt - pntArray(searchIndex));
    for ind = searchIndex + 1 : length(pntArray)
        dist = norm(pnt - pntArray(ind));
        if dist < distClosest
            distClosest = dist;
            indexClosest = ind;
        end
    end
end
