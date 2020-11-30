function extrinsics = json_cad_model(cad_model, extrinsics_filename)

params.yaw_ximea_tilt = cad_model(1);
params.pitch_ximea_tilt = cad_model(2);
params.roll_ximea_tilt = cad_model(3);
params.x_ximea_tilt = cad_model(4);
params.y_ximea_tilt = cad_model(5);
params.z_ximea_tilt = cad_model(6);
params.x_tilt_pan = cad_model(7);
params.y_tilt_pan = cad_model(8);
params.z_tilt_pan = cad_model(9);
params.yaw_base_imperx = cad_model(10);
params.pitch_base_imperx = cad_model(11);
params.roll_base_imperx = cad_model(12);
params.x_base_imperx = cad_model(13);
params.y_base_imperx = cad_model(14);
params.z_base_imperx = cad_model(15);

output.cad_model = params;
output.Comments = [char(datetime(now,'ConvertFrom','datenum')) ': CAD model generated from ' extrinsics_filename];

json_str = jsonencode(output);

filename = @(i) ['../python/cad_models/cad_model' num2str(i) '.json'];
filei = 0;
while (exist(filename(filei), 'file') == 2)
    filei = filei+1;
end
filename(filei)
fid = fopen(filename(filei), 'w');
fwrite(fid, json_str, 'char');
fclose(fid);

end
