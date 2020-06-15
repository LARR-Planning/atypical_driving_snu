function midPoint = findMidPoint(midPoints, ts, t)
    i_ts = 1;
    while ts(i_ts) < t && i_ts <= size(ts, 1)
        i_ts = i_ts + 1;
    end
    
    if i_ts == 1
        midPoint = midPoints(i_ts,:);
    elseif i_ts == size(ts, 1) + 1
        midPoint = midPoints(i_ts-1,:);
    else
        alpha = (t - ts(i_ts-1))/(ts(i_ts) - ts(i_ts-1));
        midPoint = alpha * midPoints(i_ts,:) + (1-alpha) * midPoints(i_ts-1,:);
    end
end

