function angle = complementaryFilter(angle_g, angle_am, beta)
    n = size(angle_g);
    angle = zeros(n);
    for i=2:n(1)
        angle(i,:) = beta.*angle_g(i,:) + (1-beta).*angle_am(i,:);
    end
end