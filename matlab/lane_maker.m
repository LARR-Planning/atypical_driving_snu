vertex = [0 0; ...
                0 72 ;...
                46 72 ; ....
                46 (72-43) ; ...
                46-9.8 72-43 ; ...
                26 0];

dl = 0.4;  % density of points to csv 
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
figure 
hold on
plot(vertex(:,1),vertex(:,2),'ko-')
plot(pointsX,pointsY,'go-','MarkerSize',1.2)
axis equal

laneTitle = 'KIAPI2.csv';
csvwrite(laneTitle,[pointsX'+0.2 pointsY'])




