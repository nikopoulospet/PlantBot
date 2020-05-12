function [start_time,end_time,duration,body_v] = createGait(duty_factor,omega,num_legs,R,stride_length)
%% we want input parameter duty factor (0.5 <-> 0.8) to demonstrate plant health
%there will be a constant body velocity, determined by motion command
%foot swing velocity will be calculated to satify v and duty factor
%stride length will be calculated to maintain a ratio of: u/sl = 1 second
%omega will be a variable input provided by motion command
%---------Do later----------
%create this script to be completely non blocking so that it can be run in
%a state machine. 
%update this caluclation 75% of the way through the gait cycle so a new
%pattern is ready at the start of the next cycle
%%
u = stride_length; % 1 sec cycle
body_v = (((1-duty_factor)*u)/duty_factor)-abs(omega)*R; 
duration = 1-duty_factor;

start_time = zeros(1,num_legs);
end_time = zeros(1,num_legs);

for i = 1:num_legs
    if i == 1
        start_time(1,i) = duty_factor;
        end_time(1,i) = duty_factor + duration;
    elseif i == 2 || i == 4 || i == 6
        start_time(1,i) = mod(start_time(1,i-1)+0.5,1);
        end_time(1,i) = mod(start_time(1,i)+duration, 1.000001);
    elseif i == 3 || i == 5 || i == 7
        start_time(1,i) = mod(start_time(1,i-2)-duration,1.000001);
        end_time(1,i) = start_time(1,i)+duration;
    end
end