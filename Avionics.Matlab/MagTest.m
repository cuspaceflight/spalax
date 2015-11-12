MagReference = [1;4;6];
MagReference = MagReference / norm(MagReference);

degToRad = 2.0 * pi / 360.0;
dcm = angle2dcm(20 * degToRad, 120 * degToRad,11* degToRad, 'xyz');

MagObservation = dcm*MagReference;

v = MagObservation + MagReference;
v = v/norm(v);
quat = [cross(v,MagReference); dot(v,MagReference)];

test2 = MagObservation'
test = quatrotate([quat(4) quat(1) quat(2) quat(3)],MagReference')