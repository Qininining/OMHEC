function Ad = adjointSE3(T)
R = T(1:3, 1:3);
t = T(1:3, 4);
Ad = [R, zeros(3); mhec.skew(t) * R, R];
end

