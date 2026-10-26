function [state, info] = update(state, stageTwist, pPrevious, pCurrent, method)
%UPDATE Consume one stage-frame twist and a pair of camera-frame positions.
% Tcs maps stage/base coordinates to camera coordinates; twist is [omega; v].
% Use the same length unit for positions, translation and velocity increments.
if nargin < 5, method = 'iekf'; end
assert(ismember(string(method), ["iekf", "fgd"]), 'Unknown method');
assert(isequal(size(stageTwist),[6 1]) && all(isfinite(stageTwist)), 'Expected finite 6x1 twist');
assert(isequal(size(pPrevious),[3 1]) && isequal(size(pCurrent),[3 1]) && ...
    all(isfinite([pPrevious;pCurrent])), 'Expected finite 3x1 points');
state.count = state.count + 1;
useFilter = strcmp(method, 'iekf');
    [residual, J, residualCovariance] = omhec.residual_model( ...
        state.Tcs, stageTwist, pPrevious, pCurrent, state.pointCovariance);
    commandedCameraTwist = omhec.adjointSE3(state.Tcs) * stageTwist;
    commandedCameraMotion = omhec.expSE3(commandedCameraTwist);
    commandedRotation = commandedCameraMotion(1:3, 1:3);
    if state.count <= state.cfg.observabilityInitializationSteps
        state.initializationJ = [state.initializationJ; J]; %#ok<AGROW>
        state.initializationResidual = [state.initializationResidual; residual]; %#ok<AGROW>
        oldSize = size(state.initializationResidualCovariance, 1);
        expandedCovariance = zeros(oldSize + 3);
        expandedCovariance(1:oldSize, 1:oldSize) = ...
            state.initializationResidualCovariance;
        expandedCovariance(oldSize + (1:3), oldSize + (1:3)) = ...
            residualCovariance;
        if state.count > 1
            % Adjacent residuals share n_k:
            % Cov(r_{k-1},r_k) = -Sigma_k R_k'.
            previousRows = oldSize - 2:oldSize;
            currentRows = oldSize + (1:3);
            expandedCovariance(previousRows, currentRows) = ...
                -state.pointCovariance * commandedRotation';
            expandedCovariance(currentRows, previousRows) = ...
                -commandedRotation * state.pointCovariance;
        end
        state.initializationResidualCovariance = expandedCovariance;
    end

    if useFilter
        Ppred = state.P + state.Q;
        if state.count <= state.cfg.observabilityInitializationSteps
            % A single point displacement supplies only three constraints
            % for the six-dimensional hand--eye state.  Accumulate several
            % differently directed motions before the first correction to
            % avoid an unobservable one-sample initialization transient.
            if state.count < state.cfg.observabilityInitializationSteps
                delta = zeros(6, 1);
                state.P = Ppred;
            else
                S = state.initializationJ * Ppred * state.initializationJ' + ...
                    state.initializationResidualCovariance;
                K = (Ppred * state.initializationJ') / S;
                delta = K * state.initializationResidual;
                I6 = eye(6);
                state.P = (I6 - K * state.initializationJ) * Ppred * ...
                    (I6 - K * state.initializationJ)' + ...
                    K * state.initializationResidualCovariance * K';
                state.P = 0.5 * (state.P + state.P');
            end
        else
            S = J * Ppred * J' + residualCovariance;
            K = (Ppred * J') / S;
            delta = K * residual;
            I6 = eye(6);
            state.P = (I6 - K * J) * Ppred * (I6 - K * J)' + ...
                K * residualCovariance * K';
            state.P = 0.5 * (state.P + state.P');
        end
    else
        % Fixed-gain visual-servoing baseline:
        %   v_bar = -lambda (J D)^+ e,  v = D v_bar,
        % followed by inverse virtual-velocity injection under the selected
        % left-perturbation convention.  D makes translation dimensionless
        % using a fixed characteristic length; no covariance or observation
        % weighting is introduced into this baseline.
        decayStateScale = blkdiag(eye(3), ...
            state.cfg.decayCharacteristicLength * eye(3));
        if state.count < state.cfg.observabilityInitializationSteps
            delta = zeros(6, 1);
        elseif state.count == state.cfg.observabilityInitializationSteps
            % One-time full stacked least-squares initialization.  The
            % fixed decay gain is applied only to subsequent recursive
            % single-step corrections.
            normalizedVelocity = -pinv(state.initializationJ * decayStateScale) * ...
                state.initializationResidual;
            virtualVelocity = decayStateScale * normalizedVelocity;
            delta = -virtualVelocity;
        else
            normalizedVelocity = -state.cfg.decayGain * ...
                omhec.truncatedSvdPseudoinverse(J * decayStateScale, ...
                    state.cfg.jfgdSvdRelativeThreshold) * residual;
            virtualVelocity = decayStateScale * normalizedVelocity;
            delta = -virtualVelocity;
        end
    end

    if all(isfinite(delta))
        state.Tcs = omhec.expSE3(delta) * state.Tcs;
    end
info.residual = residual;
info.jacobian = J;
info.initialized = state.count >= state.cfg.observabilityInitializationSteps;
end
