function Tsc = solve_position_only_ls(stagePoses, cameraPoints)
%SOLVE_POSITION_ONLY_LS Estimate the hand--eye transform from 3-D positions.
% STAGEPOSES is 4-by-4-by-N, CAMERAPOINTS is 3-by-N, and the returned TSC
% maps camera coordinates to stage coordinates. This is PO-LS in the paper.
% It implements the position constraint of Grossmann and Krueger over all
% samples supplied by the caller.
%
%   t_A = R_sc p_c + t_sc - R_A t_oe.
%
% R_sc is first recovered from the joint linear system and projected onto
% SO(3). With that corrected rotation fixed, t_sc and t_oe are recomputed
% by least squares as required by the paper.

numSamples = size(cameraPoints, 2);
if size(stagePoses, 3) ~= numSamples
    error('Stage-pose and camera-point sample counts must agree.');
end

rotationSystem = zeros(3 * numSamples, 15);
rightHandSide = zeros(3 * numSamples, 1);
for k = 1:numSamples
    rows = (3 * k - 2):(3 * k);
    RA = stagePoses(1:3, 1:3, k);
    tA = stagePoses(1:3, 4, k);
    rotationSystem(rows, :) = [kron(cameraPoints(:, k)', eye(3)), ...
        eye(3), -RA];
    rightHandSide(rows) = tA;
end
if rank(rotationSystem) < 15
    error('The position-only rotation system is rank deficient.');
end

solution = rotationSystem \ rightHandSide;
Rraw = reshape(solution(1:9), 3, 3);
[U, ~, V] = svd(Rraw);
Rsc = U * V';
if det(Rsc) < 0
    Rsc = U * diag([1, 1, -1]) * V';
end

translationSystem = zeros(3 * numSamples, 6);
translationRightHandSide = zeros(3 * numSamples, 1);
for k = 1:numSamples
    rows = (3 * k - 2):(3 * k);
    RA = stagePoses(1:3, 1:3, k);
    tA = stagePoses(1:3, 4, k);
    translationSystem(rows, :) = [eye(3), -RA];
    translationRightHandSide(rows) = tA - Rsc * cameraPoints(:, k);
end
if rank(translationSystem) < 6
    error('The position-only translation system is rank deficient.');
end
translation = translationSystem \ translationRightHandSide;
tSc = translation(1:3);

Tsc = [Rsc, tSc; 0, 0, 0, 1];
end
