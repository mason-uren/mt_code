
% 2020-09-20  Yang Chen
% Given a calibrated dynamic extrinsics model (a.k.a. "cad-model")
%
%  cad_model=[rx_ptu, ry_ptu, rz_ptu, x_ptu, y_ptu, z_ptu, pan, tilt, ...
%    rx_camera, ry_camera, rz_camera, x_camera, y_camera, z_camera]
% 
% and an arbitrary 3D point in the word (Imperx) coordinate system,
% we want to find out what are the pan & tilt angles needed for the PTU
% to aim the camera at the point so that the image of the image falls
% at the center of the image.

syms rx_ptu ry_ptu rz_ptu x_ptu y_ptu z_ptu
syms pan tilt rx_camera ry_camera rz_camera x_camera y_camera z_camera

% camera intrinsics:
syms fx fy u0 v0

% Point in world coordinates, Pw:
syms Xw Yw Zw

Mptu = rotation_matrix(rx_ptu, ry_ptu, rz_ptu)*translation_matrix(x_ptu, y_ptu, z_ptu);
Mpantilt = rotation_matrix(tilt, pan, 0); % assuming rz_offset = z_offset = 0 in cad-model
Mcamera = rotation_matrix(rx_camera, ry_camera, rz_camera)* ...
            translation_matrix(x_camera, y_camera, z_camera);
        
% Dynamic Extrinsic matrix:
DE = Mcamera * Mpantilt * Mptu;

% Pw in camera coordinates, Pc:
Pc = DE(1:3,1:4) * [ Xw Yw Zw 1]';

% Projection using camera matrix to get (u,v), which we want to be (0, 0): 
% [u, v]' = [ fx, 0, u0; 0, fy, v0] * Pw;
% sol = solve([ fx, 0, u0; 0, fy, v0] * Pc == Pc(3)*[u0; v0], pan, tilt)
% Or simply:
[sol_pan, sol_tilt] = solve([Pc(1) == 0, Pc(2) == 0], [pan, tilt])

%sol_tilt = solve(Pc(1) == 0, tilt)
