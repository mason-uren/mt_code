function T = computeSE3( rvec, tvec )

T(1:3,1:3) = rotationVectorToMatrix(rvec);
T(1:3,4) = tvec;
T(4,1:4) = [0 0 0 1];

end

