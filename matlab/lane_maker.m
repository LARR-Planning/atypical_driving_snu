vertex = [0 0; ...
                0 75 ;...
                -60 75 ; ....
                -60 -5];

dl = 0.05;  % density of points to csv 
pointsX = [];
pointsY = [];

for n = 1:length(vertex)-1
    lCur = norm(vertex(n+1,:)-vertex(n,:));
    Npnts = floor(lCur/dl);
    xpoints = linspace(vertex(n,1),vertex(n+1,1),Npnts);
    ypoints = linspace(vertex(n,2),vertex(n+1,2),Npnts);
    pointsX = [pointsX xpoints(1:end-1)];
    pointsY = [pointsY ypoints(1:end-1)];    
end
smoothingFactor = 500;
pointsX = smooth(pointsX,smoothingFactor);
pointsY = smooth(pointsY,smoothingFactor);

figure 
hold on
plot(vertex(:,1),vertex(:,2),'ko-')
plot(pointsX,pointsY,'go-','MarkerSize',1.2)
axis equal
%%
laneTitle = 'KIAPI3.csv';
csvwrite(laneTitle,[reshape(pointsX,[],1) reshape(pointsY,[],1)])





