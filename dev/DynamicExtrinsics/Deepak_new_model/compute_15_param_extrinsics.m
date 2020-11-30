function extrinsics = compute_15_param_extrinsics(cad_model, tilt, pan)
%% Deepak Khosla, May 31, 2019
% Compute symbolic and numerical extrinsics from groundtruth cad_model
% Ximea -> Tilt -> Pan -> Base -> Imperx

%% Numerical Extrinsics
% Ximea to Tilt
yaw_ximea_tilt=cad_model(1);
pitch_ximea_tilt=cad_model(2);
roll_ximea_tilt=cad_model(3);
x_ximea_tilt=cad_model(4);
y_ximea_tilt=cad_model(5);
z_ximea_tilt=cad_model(6);
% Tilt to Pan
pitch_tilt_pan=tilt;
x_tilt_pan=cad_model(7);
y_tilt_pan=cad_model(8);
z_tilt_pan=cad_model(9);
% Pan to Base
yaw_pan_base=pan;
% Base to Imperx
yaw_base_imperx=cad_model(10);
pitch_base_imperx=cad_model(11);
roll_base_imperx=cad_model(12);
x_base_imperx=cad_model(13);
y_base_imperx=cad_model(14);
z_base_imperx=cad_model(15);

% Throughout the remaining code, t_ denotes a translation, whereas r_ denotes
% a rotation. Finally, T_ denotes an arbitrary transformation

% Ximea to Tilt (ximea_tilt)
r_ximea_tilt = rotation_matrix(yaw_ximea_tilt, pitch_ximea_tilt, roll_ximea_tilt);
t_ximea_tilt = translation_matrix(x_ximea_tilt, y_ximea_tilt, z_ximea_tilt);

% Tilt to Pan (tilt_pan)
r_tilt_pan = rotation_matrix(0, pitch_tilt_pan, 0);
t_tilt_pan = translation_matrix(x_tilt_pan, y_tilt_pan, z_tilt_pan);

% Pan to Base (pan_base)
r_pan_base = rotation_matrix(yaw_pan_base, 0, 0);

% Base to Imperx (base_imperx)
r_base_imperx = rotation_matrix(yaw_base_imperx, pitch_base_imperx, roll_base_imperx);
t_base_imperx = translation_matrix(x_base_imperx, y_base_imperx, z_base_imperx);
                               
% Full transformation Ximea to Imperx (ximea_imperx), Yang's order
extrinsics = r_base_imperx * t_base_imperx * r_pan_base * t_tilt_pan ...
           * r_tilt_pan * t_ximea_tilt * r_ximea_tilt ;

end
