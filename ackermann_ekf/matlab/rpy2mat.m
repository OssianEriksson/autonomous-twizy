function M = rpy2mat(roll, pitch, yaw)
    cr = cos(roll);  sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw);   sy = sin(yaw);
    
    M = [cy*cp, cy*sp*sr - cr*sy, sy*sr + cy*cr*sp;
         cp*sy, cy*cr + sy*sp*sr, cr*sy*sp - cy*sr;
         -sp,   cp*sr,            cp*cr           ];
end