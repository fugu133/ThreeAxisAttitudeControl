function q_conj = qConj(q) 
    q_conj = [q(:,1), -q(:,2:4)];
end
