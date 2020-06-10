function [p_closest, minDist] = getClosestPointToLine(p, a_, b_)
    a = a_ - p;
    b = b_ - p;
    
    minDist = norm(a);
    p_closest = a_;
    
    dist_b = norm(b);
    if minDist > dist_b
        minDist  = dist_b;
        p_closest = b_;
    end
    
    n_line = (b-a) / norm(b-a);
    c = a - n_line * dot(a, n_line);
    dist_c = norm(c);
    if(dot(c-a, c-b) < 0 && minDist > dist_c)
        minDist  = dist_c;
        p_closest = c + p;
    end
end

