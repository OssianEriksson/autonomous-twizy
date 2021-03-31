function [roll, pitch, yaw] = mat2rpy(M)
    roll  = atan2(M(3,2), M(3,3));
    pitch = -asin(M(3,1));
    yaw   = atan2(M(2,1), M(1,1));
end