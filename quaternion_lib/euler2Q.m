function q = eul2Q(angle, rot_type)
    R = euler2Rotmat(angle, rot_type);
    q = rotmat2Q(R);
end