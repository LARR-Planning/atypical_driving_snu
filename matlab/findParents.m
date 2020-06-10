function parents = findParents(map, laneTree, id, midPoint)
    if id == 1
        parents = 0;
        return;
    end

    if midPoint == [11.25 1]
        debug = 1;
    end
    
    parents = [];
    i_tree = size(laneTree, 1);
    while i_tree > 0 && laneTree(i_tree).id >= id - 2
        if laneTree(i_tree).id == id
            i_tree = i_tree - 1;
            continue;
        end
        parentPoint = laneTree(i_tree).midPoint;
        delta = midPoint - parentPoint;
        collisionPoint = [NaN NaN];
        if parentPoint(1) > map.XWorldLimits(1) && parentPoint(1) < map.XWorldLimits(2) && parentPoint(2) > map.YWorldLimits(1) && parentPoint(2) < map.YWorldLimits(2)
            collisionPoint = rayIntersection(map, [parentPoint, 0], atan2(delta(2), delta(1)), norm(delta) + 0.01);
        end
        if isnan(collisionPoint(1))
            parents = [parents i_tree];
        end
        i_tree = i_tree - 1;
    end
end

