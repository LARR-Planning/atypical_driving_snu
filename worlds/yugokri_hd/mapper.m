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

%% Sectioning - use Display Tip along white curve from above
% .... do fucking things from mouse clicking and export variable 
knots = [startBridge startPlane startHell startSidePark startNarrow startNormalTwoLane startForest];
save('section_division_pnts.mat','knots')


%% Sectioning knots and bind index of lane 
data = load('section_division_pnts.mat');
knotPnts = (data.knots); nKnot = length(knotPnts);
knotsAug = knotPnts; % to close loop
knotsAug(nKnot+1) = knots(1);
sectionNames = {'bridge','plane','hell','side_park','narrow','normal_two_lane','forest'};
searchStart = 1;
for n = 1:nKnot + 1
    pnt = knotsAug(n).Position(1:2);
    ind = getClosestIndex(pnt,lane,searchStart);
    knotWithIndex(n).pnt = pnt;
    knotWithIndex(n).ind = ind; 
    if n < nKnot
        fprintf('Start point of %s = [%f,%f] at index %d\n',...
            sectionNames{n},knotWithIndex(n).pnt(1),knotWithIndex(n).pnt(2),knotWithIndex(n).ind);
    end
end

%% Visualize knots 
for n = 1:nKnot
    pnt = knotWithIndex(n).pnt;
    ind = knotWithIndex(n).ind;    
    plot3(pnt(1),pnt(2),zOffSet,'ro','MarkerFaceColor','r')
end

%% Visualize sections 
narrow = 3.5; normal = 4; wide = 5;
laneWidthSet = [wide, wide, narrow, normal,narrow,wide,narrow];
colormapJet = jet;
nColor = length(colormapJet);
colorPickIdx = linspace(1,nColor,(nKnot));
colorSection = colormapJet(max(floor(colorPickIdx),1),:);

% Identify sections
for sectionIdx = 1:nKnot
   ind1 = knotWithIndex(sectionIdx).ind;
   ind2 = knotWithIndex(sectionIdx+1).ind;
   if ind1 < ind2
       indAlongSection{sectionIdx} = ind1:ind2;
   else
      indAlongSection{sectionIdx} = [ind1:length(lane) 1:ind2];
   end
   pntAlongSection{sectionIdx} = lane(indAlongSection{sectionIdx} ,:);
   
   pnts3d = [pntAlongSection{sectionIdx} zOffSet*ones(size(pntAlongSection{sectionIdx},1),1)];
   hh = plot3(pnts3d(:,1),pnts3d(:,2),pnts3d(:,3),'LineWidth',4);
   hh.Color(1:3) = colorSection(sectionIdx,:);
end


%% Subfunctions

function indexClosest = getClosestIndex(pnt,pntArray,searchIndex)
    % pntArray = [Narray x dim]
    % pnt = [1 x dim]
    % searchIndex = Lookup from this index 
    
    indexClosest = searchIndex;
    distClosest = norm(pnt - pntArray(searchIndex));
    for ind = searchIndex + 1 : length(pntArray)
        dist = norm(pnt - pntArray(ind,:));
        if dist < distClosest
            distClosest = dist;
            indexClosest = ind;
        end
    end
end
