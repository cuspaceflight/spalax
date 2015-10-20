format long

%%Setup Matrices
MagReference = [sym('MR0');sym('MR1');sym('MR2')];
AccelReference = [sym('AR0');sym('AR1');sym('AR2')];

MagObservation = [sym('MO0');sym('MO1');sym('MO2')];
AccelObservation = [sym('AO0');sym('AO1');sym('AO2')];

%MagObservation = (MagObservation/norm(MagObservation));
%AccelObservation = (AccelObservation/norm(AccelObservation));

magA = sym('MagA');
accelA = sym('AccelA');

%%Compute Quaternion

% This method becomes less accurate (tending to a singularity) as the angle
% about any axis tends to pi (i.e gamma tends to 0)
% This can be dealt by rotating the reference vectors by pi about the
% problematic axis and then rotating the quaternion back at the end when
% gamma is below a certain threshold value
% This has not been implemented here but might be implemented on the rocket

%B = accelA*AccelObservation*transpose(AccelReference) + magA*MagObservation*transpose(MagReference)
B = sym('B', [3 3]);

%S = B + transpose(B)
S = sym('S',[3 3]);

sigma = sym('sigma');%magA*dot(MagObservation,MagReference) + accelA*dot(AccelObservation,AccelReference)

Z = [(B(2,3)-B(3,2)); (B(3,1)-B(1,3)); (B(1,2)-B(2,1))]

I3 = [1 0 0; 0 1 0; 0 0 1];

deltaCos = dot(MagObservation,AccelObservation)*dot(MagReference,AccelReference) + norm(cross(MagObservation,AccelObservation))*norm(cross(MagReference,AccelReference))

lambda = sym('lambda');%sqrt(magA^2 + 2*magA*accelA*deltaCos + accelA^2)

%Y = (lambda+sigma)*I3-S
%Y = inv(Y)
Y = sym('Y',[3 3]);
Y = Y*Z

%Q3 = [Y; 1] / sqrt(norm(Y)*norm(Y) + 1)

%[yaw, pitch, roll] = EulerAngles(Q3)



