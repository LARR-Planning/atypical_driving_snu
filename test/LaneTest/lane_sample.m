% Scenario : L shape road in airsim 

N1 = 20;
N2 = 10;
lane1 = [zeros(1,N1) ; linspace(0,73,N1)];
lane2 = [ linspace(0,49,N2) ; ones(1,N2)*73];

% save it to csv file 
lane = [lane1 lane2];

csvwrite('waypoint_airsim.csv',lane');

