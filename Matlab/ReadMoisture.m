function [traj_sel,duty_factor,timing,searching] = ReadMoisture()
    %simulate reading of sensor data
    sensorRead = 0.1;
    searching = 0; %% set to one if plant is searching for sunlight
    if sensorRead <=0.15 %desprate enable dance
        traj_sel = 0;
        duty_factor = 0.75;
        timing = 3;
    elseif sensorRead <= 0.33
        traj_sel = 1; %box LT
        duty_factor = 0.875; %almost static
        timing = 3; %
    elseif sensorRead >= 0.66
        traj_sel = 3; %trapezoid LT
        duty_factor = 0.5; %tripod -> fastest gait : 'healtiest' motion
        timing = 5; %
    else
        traj_sel = 2; %triangle LT
        duty_factor = 0.75; %halfway
        timing = 4; %
    end
end