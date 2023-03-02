function sig = lowpassFilterFo(raw, alpha)
    n = size(raw,1);
    sig(1,:) = raw(1,:);
    
    for i = 2:n
        sig(i,:) = alpha*raw(i,:) + (1-alpha)*sig(i-1,:);
    end
end