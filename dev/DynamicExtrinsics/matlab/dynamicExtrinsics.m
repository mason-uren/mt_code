function tf = dynamicExtrinsics( t, p, params )

%% Camera to Tilt
% transformation from camera to tilt frame (pitch)
tf.t_T_c = [1          0           0           params.px_ct;
            0          cos(t)     -sin(t)      params.py_ct;
            0          sin(t)      cos(t)      params.pz_ct;
            0          0           0           1           ];
     
%% Tilt to Pan
% transformation from tilt to pan frame (yaw)
tf.p_T_t = [cos(p)     0           sin(p)      params.px_tp;
            0          1           0           params.py_tp;
           -sin(p)     0           cos(p)      params.pz_tp;
            0          0           0           1           ];
     
%% Pan to World
% transformation for pan to world frame
Rx = [1                     0                       0                ;
      0                     cos(params.rx_pw)      -sin(params.rx_pw);
      0                     sin(params.rx_pw)       cos(params.rx_pw)];
  
Ry = [cos(params.ry_pw)     0                       sin(params.ry_pw);
      0                     1                       0                ;
     -sin(params.ry_pw)     0                       cos(params.ry_pw)];
 
Rz = [cos(params.rz_pw)    -sin(params.rz_pw)       0                ;
      sin(params.rz_pw)     cos(params.rz_pw)       0                ;
      0                     0                       1                ];
  
tf.w_T_p(1:3, 1:3) = Rz*Ry*Rx;
tf.w_T_p(1:3, 4) = [params.px_pw; params.py_pw; params.pz_pw];
tf.w_T_p(4, 1:4) = [0, 0, 0, 1];

%% Camera to World
tf.w_T_c = tf.w_T_p * tf.p_T_t * tf.t_T_c;

end
