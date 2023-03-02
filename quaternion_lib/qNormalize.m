function q_n = qNormalize(q)
    n = size(q,1);
    q_n = zeros(n,4);
    for k = 1:n
        q_n(k,:) = q(k,:)/norm(q(k,:));
    end
end