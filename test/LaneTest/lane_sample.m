% Scenario : L shape road in airsim 

N1 = 20;
N2 = 10;
lane1 = [zeros(1,N1) ; linspace(0,73,N1)];
lane2 = [ linspace(0,49,N2) ; ones(1,N2)*73];

% save it to csv file 
lane = [lane1 lane2];
lane = [lane [53  57 61 ; 73 73 73]]; % Yunwoo and Jungwon
csvwrite('waypoint_airsim.csv',lane');

mat = load('waypoint.csv');
mat_new = float(mat(1:3:end,:));
dlmwrite('waypoint_keti_sparse.csv',mat_new,'precision', '%.3f');


%% load 

orig_lane = load('lane/waypoint_airsim.csv');
csvwrite('lane/waypoint_airsim_new.csv',orig_lane);


