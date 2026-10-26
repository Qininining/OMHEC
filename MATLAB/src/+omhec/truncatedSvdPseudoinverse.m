function Aplus = truncatedSvdPseudoinverse(A, relativeThreshold)
%TRUNCATEDSVDPSEUDOINVERSE Suppress weak instantaneous PR-FGD directions.
% The cutoff is relative to the largest singular value of the current
% dimensionless single-observation Jacobian. The six-observation stacked
% initializer deliberately continues to use the ordinary pseudoinverse.
[U, S, V] = svd(A, 'econ');
singularValues = diag(S);
Aplus = zeros(size(A, 2), size(A, 1));
if isempty(singularValues) || singularValues(1) <= 0
    return;
end
keep = singularValues > relativeThreshold * singularValues(1);
if any(keep)
    Aplus = V(:, keep) * diag(1 ./ singularValues(keep)) * U(:, keep)';
end
end

