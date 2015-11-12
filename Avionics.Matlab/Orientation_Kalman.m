% This was used to generate the large chunks of code found within
% orientation_kalman.h
%
% It doesn't perform any purpose other than being a working environment
% for symbolic evaluation of the algorithm

%{ 
% Matrix Multiplication
a = [sym('a[0][0]') sym('a[0][1]') sym('a[0][2]');
    sym('a[1][0]') sym('a[1][1]') sym('a[1][2]');
    sym('a[2][0]') sym('a[2][1]') sym('a[2][2]')];

b = [sym('b[0][0]') sym('b[0][1]') sym('b[0][2]');
    sym('b[1][0]') sym('b[1][1]') sym('b[1][2]');
    sym('b[2][0]') sym('b[2][1]') sym('b[2][2]')];

Temp = a*b

for i = 1:3
    for j = 1:3
        fprintf('out[%i][%i] = %s;\n',i-1,j-1, char(Temp(i,j)));
    end
end

a = [sym('a[0][0]') sym('a[0][1]') sym('a[0][2]') sym('a[0][3]');
    sym('a[1][0]') sym('a[1][1]') sym('a[1][2]') sym('a[1][3]');
    sym('a[2][0]') sym('a[2][1]') sym('a[2][2]') sym('a[2][3]');
    sym('a[3][0]') sym('a[3][1]') sym('a[3][2]') sym('a[3][3]')];
    
b = [sym('b[0][0]') sym('b[0][1]') sym('b[0][2]') sym('b[0][3]');
    sym('b[1][0]') sym('b[1][1]') sym('b[1][2]') sym('b[1][3]');
    sym('b[2][0]') sym('b[2][1]') sym('b[2][2]') sym('b[2][3]');
    sym('b[3][0]') sym('b[3][1]') sym('b[3][2]') sym('b[3][3]')];

Temp = a*b

for i = 1:4
    for j = 1:4
        fprintf('out[%i][%i] = %s;\n',i-1,j-1, char(Temp(i,j)));
    end
end
%}

%% Test Area

%{
T = [-0.348747998 -0.246521398 0.0912816748 7.31283331;
    -0.339946687 -0.238602221 0.0887230188 7.10779285;
    -0.336795211 -0.237387925 0.0889075547 7.04189539;
    -0.153386071 -0.108114280 0.0400328860 3.21036959];
    
Test = det(T)
    
    
T = [sym('hphrk_inv[0][0]') sym('hphrk_inv[0][1]') sym('hphrk_inv[0][2]') sym('hphrk_inv[0][3]');
    sym('hphrk_inv[1][0]') sym('hphrk_inv[1][1]') sym('hphrk_inv[1][2]') sym('hphrk_inv[1][3]');
    sym('hphrk_inv[2][0]') sym('hphrk_inv[2][1]') sym('hphrk_inv[2][2]') sym('hphrk_inv[2][3]')
    sym('hphrk_inv[3][0]') sym('hphrk_inv[3][1]') sym('hphrk_inv[3][2]') sym('hphrk_inv[3][3]')];
inverse = inv(T)
simplify(T);
for i = 1:4
    for j = 1:4
        fprintf('k[%i][%i] = %s;\n',i-1,j-1, char(inverse(i,j)));
    end
end
%}
%% Setup

x = [sym('x[0]'); sym('x[1]'); sym('x[2]'); sym('x[3]'); sym('x[4]'); sym('x[5]'); sym('x[6]')];

edt0 = sym('edt0');
edt1 = sym('edt1');
edt2 = sym('edt2');
d = sym('d');

phi = [edt0 0 0 0 0 0 0; 
    0 edt1 0 0 0 0 0; 
    0 0 edt2 0 0 0 0; 
    -(x(5)*d/2) -(x(6)*d/2) -(x(7)*d/2) 1 -(x(1)*d/2) -(x(2)*d/2) -(x(3)*d/2)
    (x(4)*d/2) -(x(7)*d/2) (x(6)*d/2) (x(1)*d/2) 1 (x(3)*d/2) -(x(2)*d/2) 
    (x(7)*d/2) (x(4)*d/2) -(x(5)*d/2) (x(2)*d/2) -(x(3)*d/2) 1 (x(1)*d/2)
    -(x(6)*d/2) (x(5)*d/2) (x(4)*d/2) (x(3)*d/2) (x(2)*d/2) -(x(1)*d/2) 1];



%% Gyro Update

disp('Gryo Update');

%H = [1 1 1 0 0 0 0];
H = [1 0 0 0 0 0 0;
    0 1 0 0 0 0 0;
    0 0 1 0 0 0 0];

P = [sym('pt[0][0]') sym('pt[0][1]') sym('pt[0][2]') sym('pt[0][3]') sym('pt[0][4]') sym('pt[0][5]') sym('pt[0][6]');
    sym('pt[1][0]') sym('pt[1][1]') sym('pt[1][2]') sym('pt[1][3]') sym('pt[1][4]') sym('pt[1][5]') sym('pt[1][6]');
    sym('pt[2][0]') sym('pt[2][1]') sym('pt[2][2]') sym('pt[2][3]') sym('pt[2][4]') sym('pt[2][5]') sym('pt[2][6]');
    sym('pt[3][0]') sym('pt[3][1]') sym('pt[3][2]') sym('pt[3][3]') sym('pt[3][4]') sym('pt[3][5]') sym('pt[3][6]');
    sym('pt[4][0]') sym('pt[4][1]') sym('pt[4][2]') sym('pt[4][3]') sym('pt[4][4]') sym('pt[4][5]') sym('pt[4][6]');
    sym('pt[5][0]') sym('pt[5][1]') sym('pt[5][2]') sym('pt[5][3]') sym('pt[5][4]') sym('pt[5][5]') sym('pt[5][6]');
    sym('pt[6][0]') sym('pt[6][1]') sym('pt[6][2]') sym('pt[6][3]') sym('pt[6][4]') sym('pt[6][5]') sym('pt[6][6]')];

Rk = [sym('Rk') 0 0;
      0 sym('Rk') 0;
      0 0 sym('Rk')];

HPHRK = (H * P * H' + Rk)
HPHRKInv = [sym('hphrk_inv[0][0]') sym('hphrk_inv[0][1]') sym('hphrk_inv[0][2]');
    sym('hphrk_inv[1][0]') sym('hphrk_inv[1][1]') sym('hphrk_inv[1][2]');
    sym('hphrk_inv[2][0]') sym('hphrk_inv[2][1]') sym('hphrk_inv[2][2]')];

PH = P * H';
KalmanGain = PH*HPHRKInv;

for i = 1:7
    for j = 1:3
        fprintf('k[%i][%i] = %s;\n',i-1,j-1, char(KalmanGain(i,j)));
    end
end

%KalmanGain = P * H' /(H * P * H' + Rk);
KalmanGain = [sym('k[0][0]') sym('k[0][1]') sym('k[0][2]');
    sym('k[1][0]') sym('k[1][1]') sym('k[1][2]');
    sym('k[2][0]') sym('k[2][1]') sym('k[2][2]');
    sym('k[3][0]') sym('k[3][1]') sym('k[3][2]');
    sym('k[4][0]') sym('k[4][1]') sym('k[4][2]');
    sym('k[5][0]') sym('k[5][1]') sym('k[5][2]');
    sym('k[6][0]') sym('k[6][1]') sym('k[6][2]')];
P = (eye(7,7)- KalmanGain*H)*P;

for i = 1:7
    for j = 1:7
        fprintf('P[%i][%i] = %s;\n',i-1,j-1, char(P(i,j)));
    end
end

Z = [sym('gyro[0]'); sym('gyro[1]'); sym('gyro[2]')];

deltax = KalmanGain*(Z - H*x)

%% Quaternion Update Single Step

disp('Quaternion Update Single Step');

H = [0 0 0 1 0 0 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];
P = [sym('pt[0][0]') sym('pt[0][1]') sym('pt[0][2]') sym('pt[0][3]') sym('pt[0][4]') sym('pt[0][5]') sym('pt[0][6]');
    sym('pt[1][0]') sym('pt[1][1]') sym('pt[1][2]') sym('pt[1][3]') sym('pt[1][4]') sym('pt[1][5]') sym('pt[1][6]');
    sym('pt[2][0]') sym('pt[2][1]') sym('pt[2][2]') sym('pt[2][3]') sym('pt[2][4]') sym('pt[2][5]') sym('pt[2][6]');
    sym('pt[3][0]') sym('pt[3][1]') sym('pt[3][2]') sym('pt[3][3]') sym('pt[3][4]') sym('pt[3][5]') sym('pt[3][6]');
    sym('pt[4][0]') sym('pt[4][1]') sym('pt[4][2]') sym('pt[4][3]') sym('pt[4][4]') sym('pt[4][5]') sym('pt[4][6]');
    sym('pt[5][0]') sym('pt[5][1]') sym('pt[5][2]') sym('pt[5][3]') sym('pt[5][4]') sym('pt[5][5]') sym('pt[5][6]');
    sym('pt[6][0]') sym('pt[6][1]') sym('pt[6][2]') sym('pt[6][3]') sym('pt[6][4]') sym('pt[6][5]') sym('pt[6][6]')];

Rk = [sym('Rk') 0 0 0;
      0 sym('Rk') 0 0;
      0 0 sym('Rk') 0;
      0 0 0 sym('Rk')];
  
HPHRk = H * P * H' + Rk;

HPHRKInv = [sym('hphrk_inv[0][0]') sym('hphrk_inv[0][1]') sym('hphrk_inv[0][2]') sym('hphrk_inv[0][3]');
    sym('hphrk_inv[1][0]') sym('hphrk_inv[1][1]') sym('hphrk_inv[1][2]') sym('hphrk_inv[1][3]');
    sym('hphrk_inv[2][0]') sym('hphrk_inv[2][1]') sym('hphrk_inv[2][2]') sym('hphrk_inv[2][3]')
    sym('hphrk_inv[3][0]') sym('hphrk_inv[3][1]') sym('hphrk_inv[3][2]') sym('hphrk_inv[3][3]')];

PH = P*H';
KalmanGain = PH*HPHRKInv;

for i = 1:7
    for j = 1:4
        fprintf('k[%i][%i] = %s;\n',i-1,j-1, char(KalmanGain(i,j)));
    end
end


%KalmanGain = P * H' /(H * P * H' + Rk);
KalmanGain = [sym('k[0][0]') sym('k[0][1]') sym('k[0][2]') sym('k[0][3]');
    sym('k[1][0]') sym('k[1][1]') sym('k[1][2]') sym('k[0][3]');
    sym('k[2][0]') sym('k[2][1]') sym('k[2][2]') sym('k[0][3]');
    sym('k[3][0]') sym('k[3][1]') sym('k[3][2]') sym('k[0][3]');
    sym('k[4][0]') sym('k[4][1]') sym('k[4][2]') sym('k[0][3]');
    sym('k[5][0]') sym('k[5][1]') sym('k[5][2]') sym('k[0][3]');
    sym('k[6][0]') sym('k[6][1]') sym('k[6][2]') sym('k[0][3]')];
P = (eye(7,7)- KalmanGain*H)*P

for i = 1:7
    for j = 1:7
        fprintf('P[%i][%i] = %s;\n',i-1,j-1, char(P(i,j)));
    end
end

Z = [sym('quat[0]'); sym('quat[1]'); sym('quat[2]'); sym('quat[3]')];

deltax = KalmanGain*(Z - H*x)

%% Quaternion Update MultiStep 1

disp('Quaternion Update MultiStep Step 1');

H = [0 0 0 1 0 0 0;
     0 0 0 0 1 0 0];
P = [sym('pt[0][0]') sym('pt[0][1]') sym('pt[0][2]') sym('pt[0][3]') sym('pt[0][4]') sym('pt[0][5]') sym('pt[0][6]');
    sym('pt[1][0]') sym('pt[1][1]') sym('pt[1][2]') sym('pt[1][3]') sym('pt[1][4]') sym('pt[1][5]') sym('pt[1][6]');
    sym('pt[2][0]') sym('pt[2][1]') sym('pt[2][2]') sym('pt[2][3]') sym('pt[2][4]') sym('pt[2][5]') sym('pt[2][6]');
    sym('pt[3][0]') sym('pt[3][1]') sym('pt[3][2]') sym('pt[3][3]') sym('pt[3][4]') sym('pt[3][5]') sym('pt[3][6]');
    sym('pt[4][0]') sym('pt[4][1]') sym('pt[4][2]') sym('pt[4][3]') sym('pt[4][4]') sym('pt[4][5]') sym('pt[4][6]');
    sym('pt[5][0]') sym('pt[5][1]') sym('pt[5][2]') sym('pt[5][3]') sym('pt[5][4]') sym('pt[5][5]') sym('pt[5][6]');
    sym('pt[6][0]') sym('pt[6][1]') sym('pt[6][2]') sym('pt[6][3]') sym('pt[6][4]') sym('pt[6][5]') sym('pt[6][6]')];

Rk = [sym('Rk') 0;
      0 sym('Rk')];
  
HPHRk = H * P * H' + Rk;


%HPHRKInv = [sym('hphrk_inv[0][0]') sym('hphrk_inv[0][1]');
%    sym('hphrk_inv[1][0]') sym('hphrk_inv[1][1]')];
PH = P*H';
%KalmanGain = PH*HPHRKInv;
KalmanGain = PH/HPHRk;
determinant = det(HPHRk);
fprintf('float det = %s\n',char(determinant));
for i = 1:7
    for j = 1:2
        fprintf('k[%i][%i] = (%s)/det;\n',i-1,j-1, char(KalmanGain(i,j)*determinant));
    end
end
fprintf('\n');

%KalmanGain = P * H' /(H * P * H' + Rk);
KalmanGain = [sym('k[0][0]') sym('k[0][1]') ;
    sym('k[1][0]') sym('k[1][1]'); 
    sym('k[2][0]') sym('k[2][1]'); 
    sym('k[3][0]') sym('k[3][1]'); 
    sym('k[4][0]') sym('k[4][1]'); 
    sym('k[5][0]') sym('k[5][1]');
    sym('k[6][0]') sym('k[6][1]')];
P = (eye(7,7)- KalmanGain*H)*P;

for i = 1:7
    for j = 1:7
        fprintf('P[%i][%i] = %s;\n',i-1,j-1, char(P(i,j)));
    end
end

Z = [sym('quat[0]'); sym('quat[1]');];

deltax = KalmanGain*(Z - H*x)

%% Quaternion Update MultiStep 2

disp('Quaternion Update MultiStep Step 2');

H = [0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];
P = [sym('pt[0][0]') sym('pt[0][1]') sym('pt[0][2]') sym('pt[0][3]') sym('pt[0][4]') sym('pt[0][5]') sym('pt[0][6]');
    sym('pt[1][0]') sym('pt[1][1]') sym('pt[1][2]') sym('pt[1][3]') sym('pt[1][4]') sym('pt[1][5]') sym('pt[1][6]');
    sym('pt[2][0]') sym('pt[2][1]') sym('pt[2][2]') sym('pt[2][3]') sym('pt[2][4]') sym('pt[2][5]') sym('pt[2][6]');
    sym('pt[3][0]') sym('pt[3][1]') sym('pt[3][2]') sym('pt[3][3]') sym('pt[3][4]') sym('pt[3][5]') sym('pt[3][6]');
    sym('pt[4][0]') sym('pt[4][1]') sym('pt[4][2]') sym('pt[4][3]') sym('pt[4][4]') sym('pt[4][5]') sym('pt[4][6]');
    sym('pt[5][0]') sym('pt[5][1]') sym('pt[5][2]') sym('pt[5][3]') sym('pt[5][4]') sym('pt[5][5]') sym('pt[5][6]');
    sym('pt[6][0]') sym('pt[6][1]') sym('pt[6][2]') sym('pt[6][3]') sym('pt[6][4]') sym('pt[6][5]') sym('pt[6][6]')];

Rk = [sym('Rk') 0;
      0 sym('Rk')];
  
HPHRk = H * P * H' + Rk;


%HPHRKInv = [sym('hphrk_inv[0][0]') sym('hphrk_inv[0][1]');
%    sym('hphrk_inv[1][0]') sym('hphrk_inv[1][1]')];
PH = P*H';
%KalmanGain = PH*HPHRKInv;
KalmanGain = PH/HPHRk;
determinant = det(HPHRk);
fprintf('float det = %s\n',char(determinant));
for i = 1:7
    for j = 1:2
        fprintf('k[%i][%i] = (%s)/det;\n',i-1,j-1, char(KalmanGain(i,j)*determinant));
    end
end
fprintf('\n');


%KalmanGain = P * H' /(H * P * H' + Rk);
KalmanGain = [sym('k[0][0]') sym('k[0][1]') ;
    sym('k[1][0]') sym('k[1][1]'); 
    sym('k[2][0]') sym('k[2][1]'); 
    sym('k[3][0]') sym('k[3][1]'); 
    sym('k[4][0]') sym('k[4][1]'); 
    sym('k[5][0]') sym('k[5][1]');
    sym('k[6][0]') sym('k[6][1]')];
P = (eye(7,7)- KalmanGain*H)*P;

for i = 1:7
    for j = 1:7
        fprintf('P[%i][%i] = %s;\n',i-1,j-1, char(P(i,j)));
    end
end

Z = [sym('quat[2]'); sym('quat[3]');];

deltax = KalmanGain*(Z - H*x)

%% Prediction Step
disp('Prediction');

phi = [edt0 0 0 0 0 0 0; 
    0 edt1 0 0 0 0 0; 
    0 0 edt2 0 0 0 0; 
    -sym('dxby2[4]') -sym('dxby2[5]') -sym('dxby2[6]') 1 -sym('dxby2[0]') -sym('dxby2[1]') -sym('dxby2[2]')
    sym('dxby2[3]') -sym('dxby2[6]') sym('dxby2[5]') sym('dxby2[0]') 1 sym('dxby2[2]') -sym('dxby2[1]') 
    sym('dxby2[6]') sym('dxby2[3]') -sym('dxby2[4]') sym('dxby2[1]') -sym('dxby2[2]') 1 sym('dxby2[0]')
    -sym('dxby2[5]') sym('dxby2[4]') sym('dxby2[3]') sym('dxby2[2]') sym('dxby2[1]') -sym('dxby2[0]') 1];



Qk = [sym('Q00') 0 0 0 0 0 0;
      0 sym('Q11') 0 0 0 0 0;
      0 0 sym('Q22') 0 0 0 0;
      0 0 0 0 0 0 0;
      0 0 0 0 0 0 0;
      0 0 0 0 0 0 0;
      0 0 0 0 0 0 0];
    
P = [sym('pt[0][0]') sym('pt[0][1]') sym('pt[0][2]') sym('pt[0][3]') sym('pt[0][4]') sym('pt[0][5]') sym('pt[0][6]');
    sym('pt[1][0]') sym('pt[1][1]') sym('pt[1][2]') sym('pt[1][3]') sym('pt[1][4]') sym('pt[1][5]') sym('pt[1][6]');
    sym('pt[2][0]') sym('pt[2][1]') sym('pt[2][2]') sym('pt[2][3]') sym('pt[2][4]') sym('pt[2][5]') sym('pt[2][6]');
    sym('pt[3][0]') sym('pt[3][1]') sym('pt[3][2]') sym('pt[3][3]') sym('pt[3][4]') sym('pt[3][5]') sym('pt[3][6]');
    sym('pt[4][0]') sym('pt[4][1]') sym('pt[4][2]') sym('pt[4][3]') sym('pt[4][4]') sym('pt[4][5]') sym('pt[4][6]');
    sym('pt[5][0]') sym('pt[5][1]') sym('pt[5][2]') sym('pt[5][3]') sym('pt[5][4]') sym('pt[5][5]') sym('pt[5][6]');
    sym('pt[6][0]') sym('pt[6][1]') sym('pt[6][2]') sym('pt[6][3]') sym('pt[6][4]') sym('pt[6][5]') sym('pt[6][6]')];

P = phi*P*phi' + Qk;

%P = simplify(P);
for i = 1:7
    for j = 1:7
        fprintf('P[%i][%i] = %s;\n',i-1,j-1, char(P(i,j)));
    end
end



