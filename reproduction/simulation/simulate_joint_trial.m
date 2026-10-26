function histories = simulate_joint_trial(cfg, seed, rotationDeg, translationMagnitude)
% Selected prior: fixed scales with left-error coordinate mapping.
cfg.priorRotationDeg = rotationDeg;
cfg.priorTranslation = translationMagnitude;
truth = makeTruth();
histories = simulateTrial(cfg, truth, seed, rotationDeg, translationMagnitude, cfg.nominalKappa, 1);
end

function histories = simulateTrial(cfg, truth, seed, rotationDeg, ...
    translationMagnitude, kappa, noiseScale)
if nargin < 7
    noiseScale = 1;
end
rng(seed, 'twister');
initialEstimate = perturbTransform(truth.Tcs, rotationDeg, ...
    translationMagnitude);
[stagePoses, stageTwists] = makeExcitation(cfg, truth.initialStagePose, ...
    omhec.inverseTransform(initialEstimate));
truePoints = makeTruePoints(truth, stagePoses);

% Common random numbers give a paired comparison: the estimators receive
% exactly the same noisy observations within a Monte Carlo trial.
rng(seed + 1000003, 'twister');
observations = addObservationNoise(cfg, truePoints, kappa, noiseScale);
for method = 1:3
    switch method
        case 1
            histories(method) = runCumulativePositionLS(cfg, truth, ...
                initialEstimate, stagePoses, observations); %#ok<AGROW>
        case 2
            histories(method) = runJacobianEstimator(cfg, truth, ...
                initialEstimate, stageTwists, observations, kappa, noiseScale, false); %#ok<AGROW>
        case 3
            histories(method) = runJacobianEstimator(cfg, truth, ...
                initialEstimate, stageTwists, observations, kappa, noiseScale, true); %#ok<AGROW>
    end
end
end

function history = runCumulativePositionLS(cfg, truth, estimate, stagePoses, observations)
history = initializeHistory(cfg, truth.Tcs, estimate);
for k = 1:cfg.numSteps
    last = k + 1;
    sampleIndices = 1:last;
    if k >= cfg.observabilityInitializationSteps
        try
            Tsc = omhec.solve_position_only_ls(stagePoses(:, :, sampleIndices), ...
                observations(:, sampleIndices));
            candidate = omhec.inverseTransform(Tsc);
            if all(isfinite(candidate), 'all')
                estimate = candidate;
            end
        catch
            % Preserve the previous estimate if a short window is singular.
        end
    end
    [history.rotation(k + 1), history.translation(k + 1)] = ...
        transformError(truth.Tcs, estimate);
end
end

function history = runJacobianEstimator(cfg, truth, estimate, stageTwists, ...
    observations, kappa, noiseScale, useFilter)
history = initializeHistory(cfg, truth.Tcs, estimate);
measurementStd = noiseScale * [cfg.sigmaXY; cfg.sigmaXY; kappa * cfg.sigmaXY];
state = omhec.initialize(estimate, cfg, diag(measurementStd.^2));
if useFilter, method='iekf'; else, method='fgd'; end
for k = 1:cfg.numSteps
    state = omhec.update(state, stageTwists(:,k), observations(:,k), observations(:,k+1), method);
    [history.rotation(k+1), history.translation(k+1)] = transformError(truth.Tcs, state.Tcs);
end
end

function history = initializeHistory(cfg, truth, estimate)
history.rotation = nan(cfg.numSteps + 1, 1);
history.translation = nan(cfg.numSteps + 1, 1);
[history.rotation(1), history.translation(1)] = ...
    transformError(truth, estimate);
end

function truth = makeTruth()
Rsc = eul2rotm(deg2rad([20, 0, -180]), 'ZYX');
truth.Tsc = [Rsc, [12; 3; 5]; 0, 0, 0, 1];
truth.Tcs = omhec.inverseTransform(truth.Tsc);
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
    twists(:, k) = omhec.adjointSE3(TscEstimate) * cameraTwist;
    poses(:, :, k + 1) = omhec.expSE3(twists(:, k)) * poses(:, :, k);
end
end

function points = makeTruePoints(truth, stagePoses)
points = nan(3, size(stagePoses, 3));
for k = 1:size(stagePoses, 3)
    featureInBase = transformPoint(stagePoses(:, :, k), truth.toolOffset);
    points(:, k) = transformPoint(truth.Tcs, featureInBase);
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
rotationPerturbation = omhec.expSE3([axis * deg2rad(rotationDeg); zeros(3, 1)]);
estimate = truth;
estimate(1:3, 1:3) = rotationPerturbation(1:3, 1:3) * truth(1:3, 1:3);
estimate(1:3, 4) = truth(1:3, 4) + direction * translationMagnitude;
end

function [rotationError, translationError] = transformError(truth, estimate)
Rdelta = estimate(1:3, 1:3)' * truth(1:3, 1:3);
cosAngle = min(1, max(-1, (trace(Rdelta) - 1) / 2));
rotationError = rad2deg(acos(cosAngle));
translationError = norm(estimate(1:3, 4) - truth(1:3, 4));
end

function point = transformPoint(T, point)
point = T(1:3, 1:3) * point + T(1:3, 4);
end

function writeSimulationSummary(cfg, convergence, initialSensitivity, anisotropy, noiseMagnitude)
rows = repmat(struct('Study', "", 'Level', NaN, 'Method', "", ...
    'RotationMeanDeg', NaN, 'RotationStdDeg', NaN, ...
    'TranslationMeanMm', NaN, 'TranslationStdMm', NaN), 0, 1);

tail = (cfg.numSteps + 2 - cfg.finalWindow):(cfg.numSteps + 1);
for method = 1:3
    rotation = squeeze(mean(convergence.rotation(tail, :, method), 1));
    translation = squeeze(mean(convergence.translation(tail, :, method), 1));
    rows(end + 1) = makeRow("convergence", cfg.numSteps, ...
        cfg.methodLabels{method}, rotation, translation); %#ok<AGROW>
end

studyNames = {"initial_rotation", "initial_translation", ...
    "anisotropy", "noise_magnitude"};
levels = {cfg.rotationLevelsDeg, cfg.translationLevels, ...
    cfg.kappaLevels, cfg.noiseScaleLevels};
values = {initialSensitivity.rotation, initialSensitivity.translation, ...
    anisotropy, noiseMagnitude};
for study = 1:numel(studyNames)
    for level = 1:numel(levels{study})
        for method = 1:3
            rotation = squeeze(values{study}.rotation(level, :, method));
            translation = squeeze(values{study}.translation(level, :, method));
            rows(end + 1) = makeRow(studyNames{study}, levels{study}(level), ...
                cfg.methodLabels{method}, rotation, translation); %#ok<AGROW>
        end
    end
end
writetable(struct2table(rows), ...
    fullfile(cfg.dataDir, 'simulation_summary.csv'));
end

function row = makeRow(study, level, method, rotation, translation)
row.Study = string(study);
row.Level = level;
row.Method = string(method);
row.RotationMeanDeg = mean(rotation, 'omitnan');
row.RotationStdDeg = std(rotation, 0, 'omitnan');
row.TranslationMeanMm = mean(translation, 'omitnan');
row.TranslationStdMm = std(translation, 0, 'omitnan');
end

