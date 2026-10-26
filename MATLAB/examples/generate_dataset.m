function data = generate_dataset(cfg, seed, rotationDeg, translationMagnitude)
%GENERATE_DATASET Example-only ground truth, excitation and noisy observations.
if nargin<1
    cfg = mhec.default_config();
    cfg.numSteps=100; cfg.rotationStep=deg2rad(1); cfg.translationStep=0.10;
    cfg.sigmaXY=0.003; cfg.nominalKappa=10;
end
if nargin<2, seed=20260818; end
if nargin<3, rotationDeg=10; end
if nargin<4, translationMagnitude=1; end
data.truth=makeTruth();
rng(seed,'twister');
data.initial=perturbTransform(data.truth.Tcs,rotationDeg,translationMagnitude);
[data.poses,data.twists]=makeExcitation(cfg,data.truth.initialStagePose,mhec.inverseTransform(data.initial));
points=makeTruePoints(data.truth,data.poses);
rng(seed+1000003,'twister');
data.points=addObservationNoise(cfg,points,cfg.nominalKappa,1);
data.pointCovariance=diag([cfg.sigmaXY cfg.sigmaXY cfg.sigmaXY*cfg.nominalKappa].^2);
data.cfg=cfg;
end

function truth = makeTruth()
a=deg2rad(20); b=deg2rad(-180);
Rsc=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1]*[1 0 0;0 cos(b) -sin(b);0 sin(b) cos(b)];
truth.Tsc = [Rsc, [12; 3; 5]; 0, 0, 0, 1];
truth.Tcs = mhec.inverseTransform(truth.Tsc);
truth.toolOffset = [-0.5; 0.0; 0.5];
truth.initialStagePose = eye(4);
truth.initialStagePose(1:3, 4) = [1; 1; 1];
end

function [poses, twists] = makeExcitation(cfg, initialPose, TscEstimate)
poses = nan(4, 4, cfg.numSteps + 1);
twists = nan(6, cfg.numSteps);
poses(:, :, 1) = initialPose;

% Deterministic persistently exciting directions used by the physical
% motion planner: six coordinate axes and eight cube vertices for
% translation, together with thirteen rotation axes.  The coprime list
% lengths generate 182 distinct translation/rotation direction pairs.
translationDirections = [ ...
     0,  0,  1;  0,  0, -1;  1,  0,  0; -1,  0,  0; ...
     0,  1,  0;  0, -1,  0;  1,  1,  1; -1,  1,  1; ...
     1, -1,  1; -1, -1,  1;  1,  1, -1; -1,  1, -1; ...
     1, -1, -1; -1, -1, -1];
rotationDirections = [ ...
     0,  0,  1;  0,  0, -1;  1,  0,  0; -1,  0,  0; ...
     0,  1,  0;  0, -1,  0;  1,  1,  0; -1, -1,  0; ...
     1, -1,  0; -1,  1,  0;  1,  0,  1;  0,  1,  1; ...
     1,  1,  1];
translationDirections = translationDirections ./ ...
    vecnorm(translationDirections, 2, 2);
rotationDirections = rotationDirections ./ ...
    vecnorm(rotationDirections, 2, 2);

% Six dimensionless, mutually orthogonal screw directions.  With the
% angular and translational blocks normalized by their respective step
% sizes, the 6-by-6 excitation matrix has rank six and condition number
% one.  Opposite translations also keep the initialization displacement
% bounded while all three rotational axes are excited.
initialRotationDirections = [ ...
    1, 0, 0; 1, 0, 0; 0, 1, 0; ...
    0, 1, 0; 0, 0, 1; 0, 0, 1];
initialTranslationDirections = [ ...
     1,  0,  0; -1,  0,  0; 0,  1,  0; ...
     0, -1,  0;  0,  0, 1; 0,  0, -1];

for k = 1:cfg.numSteps
    if k <= cfg.observabilityInitializationSteps
        velocity = cfg.translationStep * ...
            initialTranslationDirections(k, :)';
        omega = cfg.rotationStep * initialRotationDirections(k, :)';
    else
        sequenceIndex = k - cfg.observabilityInitializationSteps - 1;
        translationIndex = mod(sequenceIndex, ...
            size(translationDirections, 1)) + 1;
        rotationIndex = mod(sequenceIndex, size(rotationDirections, 1)) + 1;
        velocity = cfg.translationStep * ...
            translationDirections(translationIndex, :)';
        omega = cfg.rotationStep * rotationDirections(rotationIndex, :)';
    end
    cameraTwist = [omega; velocity];
    twists(:, k) = mhec.adjointSE3(TscEstimate) * cameraTwist;
    poses(:, :, k + 1) = mhec.expSE3(twists(:, k)) * poses(:, :, k);
end
end

function points = makeTruePoints(truth, stagePoses)
points = nan(3, size(stagePoses, 3));
for k = 1:size(stagePoses, 3)
    featureInBase = exampleTransformPoint(stagePoses(:, :, k), truth.toolOffset);
    points(:, k) = exampleTransformPoint(truth.Tcs, featureInBase);
end
end

function observations = addObservationNoise(cfg, points, kappa, noiseScale)
stdVector = noiseScale * [cfg.sigmaXY; cfg.sigmaXY; kappa * cfg.sigmaXY];
observations = points + stdVector .* randn(size(points));
end

function estimate = perturbTransform(truth, rotationDeg, translationMagnitude)
axis = randn(3, 1);
axis = axis / norm(axis);
direction = randn(3, 1);
direction = direction / norm(direction);
rotationPerturbation = mhec.expSE3([axis * deg2rad(rotationDeg); zeros(3, 1)]);
estimate = truth;
estimate(1:3, 1:3) = rotationPerturbation(1:3, 1:3) * truth(1:3, 1:3);
estimate(1:3, 4) = truth(1:3, 4) + direction * translationMagnitude;
end

function point = exampleTransformPoint(T, point)
point=T(1:3,1:3)*point+T(1:3,4);
end
