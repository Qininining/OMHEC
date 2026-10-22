function [data, observationCovariance] = load_validation_results(cfg, magnification)
%LOAD_VALIDATION_RESULTS Recompute paired physical validation metrics.
% Each row represents one method in one repeated round. All three methods
% within a round use the same ten physical validation observations.

if nargin < 2
    magnification = '5X';
end
dataRoot = physicalDataRoot(cfg, magnification);

if ~isfolder(dataRoot)
    error('Physical experiment directory not found: %s', dataRoot);
end
addpath(cfg.analysisDir);

observationCovariance = readStaticCovariance(dataRoot, magnification);
files = dir(fullfile(dataRoot, ...
    sprintf('%s_R*_SharedValidation_N*.csv', magnification)));
rows = repmat(emptyRow(), 0, 1);
for fileIndex = 1:numel(files)
    token = regexp(files(fileIndex).name, ...
        ['^', magnification, '_R(\d+)_(PO-LS|J-FGD|J-RIEKF)_SharedValidation_'], ...
        'tokens', 'once');
    if isempty(token)
        continue;
    end

    filePath = fullfile(files(fileIndex).folder, files(fileIndex).name);
    record = read_experiment_csv(string(filePath));
    if ~isfield(record.Sections, 'ALL_DATA')
        error('ALL_DATA section missing from %s.', filePath);
    end
    metrics = recomputeMetrics(record.Sections.ALL_DATA, observationCovariance);

    row = emptyRow();
    row.Magnification = string(magnification);
    row.Round = str2double(token{1});
    methodIndex = find(strcmp(cfg.methodFileTags, token{2}), 1);
    if isempty(methodIndex)
        error('Unknown method tag %s in %s.', token{2}, files(fileIndex).name);
    end
    row.Method = string(cfg.methodLabels{methodIndex});
    row.FileName = string(files(fileIndex).name);
    row.RelativePath = "data/" + string(magnification) + "/" + row.FileName;
    row.SampleCount = metrics.SampleCount;
    row.RmseUm = metrics.RmseUm;
    row.ResidualXRmseUm = metrics.ResidualXRmseUm;
    row.ResidualYRmseUm = metrics.ResidualYRmseUm;
    row.ResidualZRmseUm = metrics.ResidualZRmseUm;
    row.NormalizedRmse = metrics.NormalizedRmse;
    row.RotationDeterminant = metrics.RotationDeterminant;
    row.RotationOrthogonalityError = metrics.RotationOrthogonalityError;
    rows(end + 1) = row; %#ok<AGROW>
end

data = struct2table(rows);
rounds = unique(data.Round);
expectedRows = numel(rounds) * 3;
if height(data) ~= expectedRows
    error('Expected %d paired validation files, found %d.', expectedRows, height(data));
end
data = sortrows(data, {'Round', 'Method'});
% Preserve acquisition round numbers for traceability; contiguity is not
% required.
if any(rounds < 1) || any(rounds ~= floor(rounds))
    error('Repeated-round indices must be positive integers.');
end
for roundIndex = rounds(:)'
    if sum(data.Round == roundIndex) ~= 3
        error('Round %d does not contain exactly three method results.', roundIndex);
    end
end
end

function covariance = readStaticCovariance(dataRoot, magnification)
files = dir(fullfile(dataRoot, sprintf('%s_StaticNoise_*.csv', magnification)));
if numel(files) ~= 1
    error('Expected one selected %s static-noise file, found %d.', ...
        magnification, numel(files));
end

record = read_experiment_csv(string(fullfile(files(1).folder, files(1).name)));
values = record.Sections.ALL_DATA.Pc_curr;
differences = diff(values, 1, 1);
covariance = zeros(3, 3);
for axis = 1:3
    % For independent stationary observations, an adjacent difference has
    % twice the variance of one observation. Use the ordinary sample
    % standard deviation of the differences and convert it back to the
    % single-observation standard deviation.
    sigma = std(differences(:, axis), 0) / sqrt(2);
    covariance(axis, axis) = sigma ^ 2;
end
end

function dataRoot = physicalDataRoot(cfg, magnification)
switch upper(magnification)
    case '5X'
        dataRoot = cfg.physicalDataRoot;
    case '10X'
        dataRoot = cfg.physicalDataRoot10X;
    case '20X'
        dataRoot = cfg.physicalDataRoot20X;
    otherwise
        error('Unsupported magnification: %s.', magnification);
end
end

function metrics = recomputeMetrics(section, covariance)
Tcs = section.T_cs_list(:, :, end);
Tsc = inverseTransform(Tcs);
sampleCount = size(section.Pc_prev, 1);
residuals = nan(sampleCount, 3);
for k = 1:sampleCount
    stageMotion = expSE3(section.Twist_s(k, :)');
    predicted = Tcs * stageMotion * Tsc * [section.Pc_prev(k, :)'; 1];
    residuals(k, :) = section.Pc_curr(k, :) - predicted(1:3)';
end
residualsUm = residuals * 1e-3;
metrics.SampleCount = sampleCount;
metrics.RmseUm = sqrt(mean(sum(residualsUm .^ 2, 2), 'omitnan'));
metrics.ResidualXRmseUm = sqrt(mean(residualsUm(:, 1) .^ 2, 'omitnan'));
metrics.ResidualYRmseUm = sqrt(mean(residualsUm(:, 2) .^ 2, 'omitnan'));
metrics.ResidualZRmseUm = sqrt(mean(residualsUm(:, 3) .^ 2, 'omitnan'));
mahalanobisSquared = nan(sampleCount, 1);
for k = 1:sampleCount
    stageMotion = expSE3(section.Twist_s(k, :)');
    deltaTransform = Tcs * stageMotion * Tsc;
    deltaRotation = deltaTransform(1:3, 1:3);
    residualCovariance = covariance + ...
        deltaRotation * covariance * deltaRotation';
    mahalanobisSquared(k) = residuals(k, :) / residualCovariance * ...
        residuals(k, :)';
end
metrics.NormalizedRmse = sqrt(mean(mahalanobisSquared, 'omitnan') / 3);
R = Tcs(1:3, 1:3);
metrics.RotationDeterminant = det(R);
metrics.RotationOrthogonalityError = norm(R' * R - eye(3), 'fro');
end

function row = emptyRow()
row = struct('Magnification', "", 'Round', NaN, 'Method', "", ...
    'FileName', "", 'RelativePath', "", 'SampleCount', 0, ...
    'RmseUm', NaN, 'ResidualXRmseUm', NaN, 'ResidualYRmseUm', NaN, ...
    'ResidualZRmseUm', NaN, 'NormalizedRmse', NaN, ...
    'RotationDeterminant', NaN, 'RotationOrthogonalityError', NaN);
end

function inverse = inverseTransform(T)
R = T(1:3, 1:3);
t = T(1:3, 4);
inverse = [R', -R' * t; 0, 0, 0, 1];
end

function T = expSE3(xi)
omega = xi(1:3);
velocity = xi(4:6);
theta = norm(omega);
Omega = skew(omega);
if theta < 1e-10
    R = eye(3) + Omega;
    V = eye(3) + 0.5 * Omega;
else
    R = eye(3) + sin(theta) / theta * Omega + ...
        (1 - cos(theta)) / theta^2 * Omega^2;
    V = eye(3) + (1 - cos(theta)) / theta^2 * Omega + ...
        (theta - sin(theta)) / theta^3 * Omega^2;
end
T = [R, V * velocity; 0, 0, 0, 1];
end

function S = skew(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
