function extrinsics = disp_cad_model(cad_model)

params.yaw_ximea_tilt = round(cad_model(1)*1000)/1000;
params.pitch_ximea_tilt = round(cad_model(2)*1000)/1000;
params.roll_ximea_tilt = round(cad_model(3)*1000)/1000;
params.x_ximea_tilt = round(cad_model(4)*1000)/1000;
params.y_ximea_tilt = round(cad_model(5)*1000)/1000;
params.z_ximea_tilt = round(cad_model(6)*1000)/1000;
params.x_tilt_pan = round(cad_model(7)*1000)/1000;
params.y_tilt_pan = round(cad_model(8)*1000)/1000;
params.z_tilt_pan = round(cad_model(9)*1000)/1000;
params.yaw_base_imperx = round(cad_model(10)*1000)/1000;
params.pitch_base_imperx = round(cad_model(11)*1000)/1000;
params.roll_base_imperx = round(cad_model(12)*1000)/1000;
params.x_base_imperx = round(cad_model(13)*1000)/1000;
params.y_base_imperx = round(cad_model(14)*1000)/1000;
params.z_base_imperx = round(cad_model(15)*1000)/1000;
disp(params);

end
