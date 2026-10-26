function state = initialize(Tcs, cfg, pointCovariance)
%INITIALIZE Initialize recursive calibration, without generating measurements.
% P is the covariance of the left correction [rotation; translation].
if nargin < 2, cfg = mhec.default_config(); end
if nargin < 3, pointCovariance = diag([0.003 0.003 0.03].^2); end
assert(isequal(size(Tcs),[4 4]) && all(isfinite(Tcs),'all'), 'Expected finite 4x4 Tcs');
assert(norm(Tcs(4,:)-[0 0 0 1]) < 1e-10 && ...
    norm(Tcs(1:3,1:3)'*Tcs(1:3,1:3)-eye(3),'fro') < 1e-8 && ...
    det(Tcs(1:3,1:3)) > 0, 'Tcs must be in SE(3)');
assert(isequal(size(pointCovariance),[3 3]) && ...
    all(isfinite(pointCovariance),'all') && ...
    norm(pointCovariance-pointCovariance','fro') < 1e-12 && ...
    all(eig(pointCovariance)>0), 'Expected positive definite point covariance');
assert(cfg.observabilityInitializationSteps >= 1 && ...
    fix(cfg.observabilityInitializationSteps)==cfg.observabilityInitializationSteps, 'Invalid initialization count');
assert(cfg.decayCharacteristicLength>0 && cfg.decayGain>=0 && ...
    cfg.jfgdSvdRelativeThreshold>=0, 'Invalid gain or scale');
state.Tcs = Tcs;
state.cfg = cfg;
state.P = mhec.initial_covariance_p0(Tcs);
state.Q = blkdiag(cfg.processRotationStd^2*eye(3), cfg.processTranslationStd^2*eye(3));
state.pointCovariance = pointCovariance;
state.count = 0;
state.initializationJ = zeros(0,6);
state.initializationResidual = zeros(0,1);
state.initializationResidualCovariance = zeros(0,0);
end
