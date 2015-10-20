if exist('SerialPort','var')
   fclose(SerialPort);
   delete(SerialPort);
end


SerialPort = serial('COM3');
set(SerialPort,'BaudRate',38400);
fopen(SerialPort);

[AccelM AccelB] = calibrate('AccelData3.txt',false,9.80665);
[MagM MagB] = calibrate('MagData4.txt',false,1.0)


%Discard any data already on buffer
fgetl(SerialPort);


% Measure Bias
i = 0;
AccelTotal = [0;0;0];
MagTotal = [0;0;0];

while true
    if (SerialPort.BytesAvailable > 64)
        string = fgetl(SerialPort);
        data = cell2mat(textscan(string,'%f'));
        %Gyro = [data(1);data(2);data(3)];
        AccelTotal = AccelTotal + AccelM*([data(4);data(5);data(6)]-AccelB);
        MagTotal = MagTotal + MagM*([data(7);data(8);data(9)]-MagB);
        i = i + 1;
        if (i == 1000)
            break;
        end
    end
end

figure(1);
graph = plot3([0 1],[0 1],[0 1]);

AccelTotal = AccelTotal/ norm(AccelTotal);
MagTotal = MagTotal/ norm(MagTotal);

xlim([-1.2 1.2])
ylim([-1.2 1.2])
zlim([-1.2 1.2])
grid on

% Read Data
while true
    if (SerialPort.BytesAvailable > 64)
        string = fgetl(SerialPort);
        data = cell2mat(textscan(string,'%f'));
        if (numel(data) ~= 9)
            continue;
        end
        Gyro = [data(1);data(2);data(3)];
        Accel = AccelM*([data(4);data(5);data(6)]-AccelB);
        Mag = MagM*([data(7);data(8);data(9)]-MagB);
        
        Q = QUEST_algorithm(AccelTotal,MagTotal,Accel/norm(Accel),Mag/norm(Mag));
        Transformed = quatrotate([Q(4) Q(1) Q(2) Q(3)],[0 0 1]);
        set(graph,'XData',[0 Transformed(1)]);
        set(graph,'YData',[0 Transformed(2)]);
        set(graph,'ZData',[0 Transformed(3)]);
        drawnow;
    end
end


fclose(SerialPort)
delete(SerialPort)
clear s