function T = expSE3(xi)
omega = xi(1:3);
velocity = xi(4:6);
theta = norm(omega);
Omega = omhec.skew(omega);
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

