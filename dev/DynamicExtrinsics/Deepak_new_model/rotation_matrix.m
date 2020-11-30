function matrix = rotation_matrix(yaw,pitch,roll)
%% Deepak Khosla, May 31, 2019
%% Jacob Moorman, June 21, 2019

% OpenCV
Rx = [1,     0,            0;
      0,     cos(pitch),  -sin(pitch);
      0,     sin(pitch),   cos(pitch)];
    
Ry = [cos(yaw),      0,   sin(yaw);
      0,             1,   0;
     -sin(yaw),      0,   cos(yaw)];

Rz = [cos(roll),   -sin(roll),   0;
      sin(roll),    cos(roll),   0;
      0,            0,           1];

matrix = [Rz*Ry*Rx    zeros(3,1);
          0 0 0       1];

end
