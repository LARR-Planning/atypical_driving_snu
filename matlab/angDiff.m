function diff = angDiff(a, b)
    diff = abs(a-b);
    if diff > pi
        diff = 2 * pi - diff;
    end
end

