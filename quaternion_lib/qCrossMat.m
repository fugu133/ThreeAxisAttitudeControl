function mat = qCrossMat(q)
    mat = [
        -q(:,2), -q(:,3), -q(:,4);
         q(:,1), -q(:,4),  q(:,3);
         q(:,4),  q(:,1), -q(:,2);
        -q(:,3),  q(:,2),  q(:,1)];
end