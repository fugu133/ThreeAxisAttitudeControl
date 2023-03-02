function angle = q2Euler(q, rot_type)
    R = q2Rotmat(q, rot_type);
    angle = rotmat2Euler(R);
end 