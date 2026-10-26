function P0 = initial_covariance_p0(initialEstimate)
% INITIAL_COVARIANCE_P0 Empirically selected prior, revalidated with Q=0.
% State correction convention: T_new = expSE3([theta; v]) * T_old.
% Units: theta in radians, v and translation in millimetres.
% The 30 deg / 0.8 mm scales are vector-magnitude scales (SD * sqrt(3)).
% First-order relation: dt = -skew(t0)*theta + v.
% These scales are fixed across all initial errors and excitation steps.
t0=initialEstimate(1:3,4);
S=[0 -t0(3) t0(2);t0(3) 0 -t0(1);-t0(2) t0(1) 0];
A=[eye(3),zeros(3);S,eye(3)];
D=blkdiag(deg2rad(30)^2/3*eye(3),0.8^2/3*eye(3));
P0=A*D*A';
P0=(P0+P0')/2;
end
