function F = solveParams( x, t, p, extrinsics )
   
%% Camera to Tilt
% transformation from camera to tilt frame (pitch)
t_T_c = [1          0           0           x(1);
         0          cos(t)     -sin(t)      x(2);
         0          sin(t)      cos(t)      x(3);
         0          0           0           1    ];
     
%% Tilt to Pan
% transformation from tilt to pan frame (yaw)
p_T_t = [cos(p)     0           sin(p)      x(4);
         0          1           0           x(5);
        -sin(p)     0           cos(p)      x(6);
         0          0           0           1    ];
     
%% Pan to World
% generic transformation for pan to world frame
Rx = [1                 0                0         ;
      0                 cos(x(10))      -sin(x(10));
      0                 sin(x(10))       cos(x(10))];
  
Ry = [cos(x(11))        0                sin(x(11));
      0                 1                0         ;
     -sin(x(11))        0                cos(x(11))];
 
Rz = [cos(x(12))       -sin(x(12))       0         ;
      sin(x(12))        cos(x(12))       0         ;
      0                 0                1         ];
  
w_T_p(1:3, 1:3) = Rz*Ry*Rx;
w_T_p(1:3, 4) = [x(7); x(8); x(9)];
w_T_p(4, 1:4) = [0, 0, 0, 1];

%% Camera to World
w_T_c = w_T_p * p_T_t * t_T_c;

%% FSolve Functions
F(1) = w_T_c(1,1) - extrinsics(1,1);
F(2) = w_T_c(1,2) - extrinsics(1,2);
F(3) = w_T_c(1,3) - extrinsics(1,3);
F(4) = w_T_c(1,4) - extrinsics(1,4);
F(5) = w_T_c(2,1) - extrinsics(2,1);
F(6) = w_T_c(2,2) - extrinsics(2,2);
F(7) = w_T_c(2,3) - extrinsics(2,3);
F(8) = w_T_c(2,4) - extrinsics(2,4);
F(9) = w_T_c(3,1) - extrinsics(3,1);
F(10) = w_T_c(3,2) - extrinsics(3,2);
F(11) = w_T_c(3,3) - extrinsics(3,3);
F(12) = w_T_c(3,4) - extrinsics(3,4);

end
