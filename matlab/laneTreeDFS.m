function tail = laneTreeDFS(laneTree, i_tree)
    if i_tree == 1
        tail = 1;
        return;
    elseif isempty(laneTree(i_tree).parents)
        tail = -1;
        return;
    end
    
    for i_parents = 1:size(laneTree(i_tree).parents, 2)
        tail_prev = laneTreeDFS(laneTree, laneTree(i_tree).parents(i_parents));
        if tail_prev == -1
            continue;
        else
            tail = [tail_prev i_tree];
            return;
        end
    end
    
    tail = -1;
end

