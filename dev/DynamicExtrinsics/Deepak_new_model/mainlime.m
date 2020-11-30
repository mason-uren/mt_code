%% Main LIME function
% Deepak Khosla, May 31, 2019
% WARNING: Requires symbolic and optimization toolboxes to run
% Sets up CAD model, computes dynamic extrinsics symbolically and solves for
% it by numerically using CAD-based ground-truth extrinsics at N PT
% positions (15 unknowns, 12 per extrinsic gives 12N equations to make it
% an over-determined system).
% Benefits
% 1. This can be used instead of John Miller's CAD model to verify HRL results
% 2. Kenny's model/code can be replaced with this one. See solveparams function

%% Clear workspace
close all;
clear all;

rng(0);

data='simulated';
% data='real';
% extrinsics_filename='Extrinsics_25_TV_High_Quality.h5';
% extrinsics_filename='Extrinsics_TV_High_Quality.h5';
% extrinsics_filename='Extrinsics.h5';
% extrinsics_filename='Extrinsics_TV_High_Quality_Checked_by_Yang.h5';

if strcmp(data,'simulated')
    %% Setup ground-truth CAD model (15 parameters)
    % Coordinate system: x right, y down, z forward, pan left, tilt down, roll ?
    % Example
%     cad_ximea_tilt=[pi/6 -pi/6 pi/4 1 0 0];     % yaw pitch roll x y z
%     cad_tilt_pan=[1 0 0];                       % x y z
%     cad_base_imperx=[pi/3 0 0 1 -1 0];          % yaw pitch roll x y z
%     cad_model=[cad_ximea_tilt cad_tilt_pan cad_base_imperx];
    cad_ximea_tilt=[0 0 0 0.35 0 0];                % yaw pitch roll x y z
    cad_tilt_pan=[0 0.2 0];                         % x y z
    cad_base_imperx=[0 0 0 -0.65 -0.2 0];           % yaw pitch roll x y z
    cad_model=[cad_ximea_tilt cad_tilt_pan cad_base_imperx];
    cad_model=0.1*randi(10,1,15);
    
    %% Display results
    disp('Exact CAD Model: ');
    disp_cad_model(cad_model);
     
    %% Numercial extrinsics
    % To solve setup, do num_pantilt_settings and get numercial extrinsics from
    % groundtruth CAD model for each setting (12 equations per setting)
    num_pantilt_settings=25;
    low=0; high=pi/2;
    pans = low + (high-low).*rand(1,num_pantilt_settings);
    tilts = low + (high-low).*rand(1,num_pantilt_settings);
    for iloop=1:num_pantilt_settings,
        extrinsics(:,:,iloop) = compute_15_param_extrinsics(cad_model, tilts(iloop), pans(iloop));
    end
end

if strcmp(data,'real')
    combined_ext = h5read(extrinsics_filename,'/combined_ext');
    pan_tilt = h5read(extrinsics_filename,'/pan_tilt')*pi/180;
    num_pantilt_settings=length(pan_tilt);
    ibad=[];
    num_valid_pantilt_settings=0;
    for i=1:num_pantilt_settings,
        if (isempty(find(ibad==i)))
            num_valid_pantilt_settings= num_valid_pantilt_settings+1;
            extrinsics(:,:,i)=transpose(combined_ext(:,:,i));
        end
    end
    num_valid_pantilt_settings
    for i=1: num_valid_pantilt_settings,
        extrinsics(4,:,i)=[0 0 0 1];
    end
    % Tranforming Scott's coordinate system (& in deg) for pan and tilt to Deepak's
    % opencv coordinate system (& in radians)
    pans=pi/2 - pan_tilt(1, :);
    tilts=-pan_tilt(2,:);
end
   
%% Initial guess
x0 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
% x0 = 0.05 + 0.1*rand(1,15);
% x0(7:8) = 0;

%% Use fsolve to find best solution
% Recover 15 parameters of CAD model
options = optimoptions('fsolve','Display','iter','Algorithm','Levenberg-Marquardt','MaxFunctionEvaluations',10000);
x = fsolve(@(x) solveparams(x,tilts,pans,extrinsics), x0, options);

% Since angles are cyclic with period 2*pi, wrap them into [-pi, pi]
x(1:3) = wrapToPi(x(1:3));
x(10:12) = wrapToPi(x(10:12));

%% Display results
disp('Calculated Parameters: ');
disp_cad_model(x);
json_cad_model(x, extrinsics_filename);

if strcmp(data,'simulated')
    cad_x=cad_ximea_tilt(4)+cad_tilt_pan(1)+cad_base_imperx(4);
    cad_y=cad_ximea_tilt(5)+cad_tilt_pan(2)+cad_base_imperx(5);
    cad_z=cad_ximea_tilt(6)+cad_tilt_pan(3)+cad_base_imperx(6);
    fprintf('%s %f %f\n','x_ximea_imperx (+ve right)  ',x(4)+x(7)+x(13),cad_x)
    fprintf('%s %f %f\n','y_ximea_imperx (+ve bottom) ',x(5)+x(8)+x(14),cad_y)
    fprintf('%s %f %f\n','z_ximea_imperx (+ve forward)',x(6)+x(9)+x(15),cad_z)
end
if strcmp(data,'real')
    fprintf('%s %f\n','x_ximea_imperx (+ve right)  ',x(4)+x(7)+x(13))
    fprintf('%s %f\n','y_ximea_imperx (+ve bottom) ',x(5)+x(8)+x(14))
    fprintf('%s %f\n','z_ximea_imperx (+ve forward)',x(6)+x(9)+x(15))
end
