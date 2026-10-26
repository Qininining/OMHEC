function [residual, J, residualCovariance] = residual_model(Tcs, stageTwist, pPrevious, pCurrent, pointCovariance)
%RESIDUAL_MODEL Position residual and analytical correction Jacobian.
    commandedCameraTwist = omhec.adjointSE3(Tcs) * stageTwist;
    commandedCameraMotion = omhec.expSE3(commandedCameraTwist);
    commandedRotation = commandedCameraMotion(1:3, 1:3);
    predicted = commandedCameraMotion(1:3, 1:3) * pPrevious + commandedCameraMotion(1:3, 4);
    residual = pCurrent - predicted;
    % For r_k = n_{k+1} - R_k n_k, the marginal covariance is
    % Sigma_{k+1} + R_k Sigma_k R_k'.  It is not exactly 2*Sigma when
    % anisotropic noise is rotated by a nonzero commanded increment.
    residualCovariance = pointCovariance + ...
        commandedRotation * pointCovariance * commandedRotation';
    omega = commandedCameraTwist(1:3);
    velocity = commandedCameraTwist(4:6);
    J = [omhec.skew(pPrevious) * omhec.skew(omega) - omhec.skew(velocity), ...
        -omhec.skew(omega)];

end
