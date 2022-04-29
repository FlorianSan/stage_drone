function q = ToQuaternion(yaw, pitch, roll)
    %Abbreviations for the various angular functions
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    q1 = cr * cp * cy + sr * sp * sy;
    q2 = sr * cp * cy - cr * sp * sy;
    q3 = cr * sp * cy + sr * cp * sy;
    q4 = cr * cp * sy - sr * sp * cy;
    
    q = [q1; q2; q3; q4];