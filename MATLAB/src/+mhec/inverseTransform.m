function inverse = inverseTransform(T)
R = T(1:3, 1:3);
t = T(1:3, 4);
inverse = [R', -R' * t; 0, 0, 0, 1];
end

