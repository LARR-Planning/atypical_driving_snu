function [p_lane, lane_index] = getCloestPointToLanePath(p, lanePath, start_index)
    minDist = inf;
    for i = start_index : size(lanePath)-1
        [p_lane_, dist] = getClosestPointToLine(p, lanePath(i,:), lanePath(i+1,:));
        if minDist > dist
            minDist = dist;
            p_lane = p_lane_;
            lane_index = i;
        end
    end
    if minDist == inf
        error("there is no cloest point!");
    end
end