function v_rot = vRotation(q, v, rot_type)
    if rot_type == "point"
        v_rot = qV(qCross(q, qCross([0, v], qConj(q))));
    else 
        v_rot = qV(qCross(qConj(q), qCross([0, v], q)));
    end
end