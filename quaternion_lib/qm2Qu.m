function q_u = qm2Qu(q_m)
    q_m = qr2Ql(q_m);
    q_u = [q_m(:,2), q_m(:,3), q_m(:,4), q_m(:,1)];
end