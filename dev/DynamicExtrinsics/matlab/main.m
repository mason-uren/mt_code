%% Load Data

% file paths
exp = '';
extrinsics.file = ['../data/' exp 'extrinsics_Ximea_to_Imperx'];
imperx.file = ['../data/' exp 'imperx_intrinsics'];
ximea.file = ['../data/' exp 'ximea_intrinsics'];

% load extrinsics data
extrinsics.rvec = h5read(extrinsics.file, '/rvec');
extrinsics.tvec = h5read(extrinsics.file, '/tvec');
extrinsics.se3 = transpose( h5read(extrinsics.file, '/composed_ext') );

extrinsics.se3(4,:) = [0 0 0 1]; % corrections
extrinsics.se3(1:2,4) = extrinsics.se3(1:2,4) * -1; 

% load imperx intrinsics data
imperx.intrinsics = transpose( h5read(imperx.file, '/intrinsics') );
imperx.fx = imperx.intrinsics(1,1); imperx.fy = imperx.intrinsics(2,2);
imperx.cx = imperx.intrinsics(1,3); imperx.cy = imperx.intrinsics(2,3);

imperx.dist = h5read(imperx.file, '/dist');

% load ximea intrinsics data
ximea.intrinsics = transpose( h5read(ximea.file, '/intrinsics') );
ximea.fx = ximea.intrinsics(1,1); ximea.fy = ximea.intrinsics(2,2);
ximea.cx = ximea.intrinsics(1,3); ximea.cy = ximea.intrinsics(2,3);

ximea.dist = h5read(ximea.file, '/dist');

%% Ideal Values

% boolean to use ideal intrinsic/extrinsics or not
use_ideal = false;

if use_ideal

    extrinsics.i_T_x = [1, 0, 0, 1.0;
                        0, 1, 0, 0.21;
                        0, 0, 1,-0.07;
                        0, 0, 0, 1  ];
                    
    imperx.intrinsics = [11130.43,  0,          2560;
                         0,         11130.43,   2560;
                         0,         0,          0   ];
                    
    imperx.fx = imperx.intrinsics(1,1); imperx.fy = imperx.intrinsics(2,2);
    imperx.cx = imperx.intrinsics(1,3); imperx.cy = imperx.intrinsics(2,3);
    
    ximea.intrinsics = [43516.48,  0,          3960;
                        0,         43507.25,   3002;
                        0,         0,          0   ];
                    
    ximea.fx = ximea.intrinsics(1,1); ximea.fy = ximea.intrinsics(2,2);
    ximea.cx = ximea.intrinsics(1,3); ximea.cy = ximea.intrinsics(2,3);

else
    
    extrinsics.i_T_x = extrinsics.se3;
    
end

%% Solve Parameters

% initial values (eyeballed)
x0 = [0.5   0.0  -0.1  -0.2  0.4   0.0   0.75  -0.2  0.0   0.0   0.0   0.0];

% tilt and pan angles
t = deg2rad(6.0588); p = deg2rad(90 - 78.944);

% use fsolve to find best solution
options = optimoptions(@fsolve,'Algorithm', 'trust-region-dogleg');
x = fsolve(@(x) solveParams(x, t, p, extrinsics.i_T_x), x0, options);

%% Extract Solution

params.px_ct = x(1); params.py_ct = x(2); params.pz_ct = x(3);
params.px_tp = x(4); params.py_tp = x(5); params.pz_tp = x(6);
params.px_pw = x(7); params.py_pw = x(8); params.pz_pw = x(9);
params.rx_pw = x(10); params.ry_pw = x(11); params.rz_pw = x(12);

%% Compute Dynamic Extrinsics

% define new pan/tilt angles
new_t = deg2rad(0); new_p = deg2rad(0);

% compute the extrinsic matrix for the specified angles
tf = dynamicExtrinsics(new_t, new_p, params);

% output results
disp('Camera to World: ');
disp(double(tf.w_T_c));

disp('Camera to Tilt: ');
disp(double(tf.t_T_c));

disp('Tilt to Pan: ');
disp(double(tf.p_T_t));

disp('Pan to World: ');
disp(double(tf.w_T_p));

disp('Calculated Parameters: ');
disp(params);

%% Test Forward Problem

% ximea
uc = 1405.431;
vc = 3949.3542;

zc = 5.5;
xc = (uc - ximea.cx) * zc / ximea.fx;
yc = (vc - ximea.cy) * zc / ximea.fy;

xyz_c = [xc; yc; zc; 1];
xyz_w = tf.w_T_c * xyz_c;

zw = xyz_w(3);
xw = xyz_w(1);
yw = xyz_w(2);

uw = (xw * imperx.fx) / zw + imperx.cx;
vw = (yw * imperx.fy) / zw + imperx.cy;

disp(['Ximea Coordinates: ' num2str([uc vc])]);
disp(['Calculated Imperx Coordinates: ' num2str([uw vw])]);
disp(['GT Imperx Coordinates: ' num2str([3243.29 2442.475])]);

%% Test Inverse Problem

