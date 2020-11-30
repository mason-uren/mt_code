function F = solveparams(cad_model, tilts, pans, all_gt_extrinsics)
%% Jacob Moorman, June 20, 2019
% Return a matrix with num_pantilt_settings rows and len(cad_model) columns.
% Each row represents the difference between the currently estimated extrinsics
% and the corresponding ground truth extrinsics. Typically, we try to optimize
% cad_model that all of the entries of this matrix are close to zero.

% Ensure the number of pantilts is the same as the number of ground truths
assert(size(tilts, 2) == size(pans, 2));
assert(size(tilts, 2) == size(all_gt_extrinsics, 3));

% Get the number of pantilts
num_pantilt_settings = size(tilts, 2);

%% Solve
for iloop=1:num_pantilt_settings,
    % Ground truth extrinsics
    gt_extrinsics = all_gt_extrinsics(:,:,iloop);
    
    % Compute the approximated extrinsics
    tilt = tilts(iloop);
    pan = pans(iloop);
    computed_extrinsics = compute_extrinsics(cad_model, tilt, pan);
    
    error = computed_extrinsics - gt_extrinsics;
    
    angle_importance = 0.5;
    error(1:3, 1:3) = angle_importance * error(1:3, 1:3);
    error(1:3, 4) = (1 - angle_importance) * error(1:3, 4);
    
    F(iloop, :) = error(:);
end

end
