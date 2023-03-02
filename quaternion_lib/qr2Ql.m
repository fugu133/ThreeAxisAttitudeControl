function q_left = qr2Ql(q_right)
    q_left = [q_right(:,1), -q_right(:,2), q_right(:,3), -q_right(:,4)];
end