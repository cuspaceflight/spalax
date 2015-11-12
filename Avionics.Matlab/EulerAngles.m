function [X, Y, Z] = EulerAngles(q)
X = -atan2(2 * (q(2) * q(3) - q(1) * q(4)), 1 - 2 * (q(1)*q(1) + q(2)*q(2)))*180/pi;
Y = asin(2 * (q(1) * q(3) + q(2) * q(4)))*180/pi;
Z = -atan2(2 * (q(1) * q(2) - q(3) * q(4)), 1 - 2 * (q(2)*q(2) + q(3)*q(3)))*180/pi;
end