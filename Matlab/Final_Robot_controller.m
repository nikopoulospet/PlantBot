clear
RG = [0.140, 0.3, 60, 60, 0.05525, 0.05525, 0.1,6];
Stride_Length = 0.04;
LP = [120*pi/180, 60*pi/180, 180*pi/180, 0*pi/180, 240*pi/180, 300*pi/180];
Router = 0.2;
Rinner = RG(1);
alpha= 60*pi/180;
R = sqrt((RG(1)+RG(5)+RG(6))^2 + RG(7)^2);
[omega,crab_angle] = MotionCommand();
[traj_sel,duty_factor,timing,searching] = ReadMoisture();
[start_time,end_time,duration,body_v] = createGait(duty_factor,omega,RG(8),R,Stride_Length);




phases = 1/duration;
dx = Stride_Length/((phases-1)*timing);
Alpha = zeros(6,(phases*timing));
Beta = zeros(6,(phases*timing));
Gamma = zeros(6,(phases*timing));

O = [0,0,0,0,0,0;
    -Stride_Length/2,-Stride_Length/2,-Stride_Length/2,-Stride_Length/2,-Stride_Length/2,-Stride_Length/2;
    0.1,0.1,0.1,0.1,0.1,0.1;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];

footPositions_B = [Router*cos(LP(1)),Router*cos(LP(2)),Router*cos(LP(3)),Router*cos(LP(4)),Router*cos(LP(5)),Router*cos(LP(6));
    Router*sin(LP(1)),Router*sin(LP(2)),Router*sin(LP(3)),Router*sin(LP(4)),Router*sin(LP(5)),Router*sin(LP(6));
    0,0,0,0,0,0];

if traj_sel ~= 0
    for i=1:(phases*timing)
        if i<(phases-1)*timing
            O = O + [0,0,0,0,0,0;dx,dx,dx,dx,dx,dx;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
        elseif i == (phases-1)*timing
            O = O - [0,0,0,0,0,0;0,0,0,0,0,0;0.025,0.025,0.025,0.025,0.025,0.025;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
            dx = Stride_Length *(1/(timing-1));
        elseif i == phases*timing
            O = O + [0,0,0,0,0,0;0,0,0,0,0,0;0.025,0.025,0.025,0.025,0.025,0.025;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
        else
            O = O - [0,0,0,0,0,0;dx,dx,dx,dx,dx,dx;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
        end
        [a, b, g]= IKwithLegs(O,RG,footPositions_B,LP);
        Alpha(:,i) = transpose(a);
        Beta(:,i) = transpose(b);
        Gamma(:,i) = transpose(g);
        
    end
    
    
    for i = 1:6
        index = round(start_time(i)*(phases*timing));
        for n = 1:(phases*timing)
            if index+n > (phases*timing)
                index = (-n)+1;
            end
            tempA(i,n) = Alpha(i,index+n);
            tempB(i,n) = Beta(i,index+n);
            tempG(i,n) = Gamma(i,index+n);
        end
    end
    Alpha = tempA
    Beta = tempB
    Gamma = tempG
    
    Search_rotation = 1/((phases*timing)/4)
    Salpha = [0,0,0,0,0,0];
    if searching == 1
        O = [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
        for i = 1:(phases*timing)
            
            if i < (phases*timing)/4
                O = O + [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;Search_rotation,Search_rotation,Search_rotation,Search_rotation,Search_rotation,Search_rotation];
            elseif i < (phases*timing)/2
                O = O - [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;Search_rotation,Search_rotation,Search_rotation,Search_rotation,Search_rotation,Search_rotation];
            elseif i < 3*(phases*timing)/4
                O = O - [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;Search_rotation,Search_rotation,Search_rotation,Search_rotation,Search_rotation,Search_rotation];
            else
                O = O + [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;Search_rotation,Search_rotation,Search_rotation,Search_rotation,Search_rotation,Search_rotation];
            end
            [a, ~, ~]= IKwithLegs(O,RG,footPositions_B,LP);
            Salpha(i) = LP(1) - a(1);
        end
        for i=1:6
            Alpha(i,:) = Alpha(i,:) + Salpha;
        end
    end
else
    O = [0,0,0,0,0,0;0,0,0,0,0,0;0.1,0.1,0.1,0.1,0.1,0.1;
        0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
    dz = 0.05/(phases*timing)*2
    for i=1:(phases*timing)
        if i < (phases*timing)/4
            O = O + [0,0,0,0,0,0;0,0,0,0,0,0;dz,dz,dz,dz,dz,dz;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
        elseif i < (phases*timing)/2
            O = O - [0,0,0,0,0,0;0,0,0,0,0,0;dz,dz,dz,dz,dz,dz;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
        elseif i < 3*(phases*timing)/4
            O = O - [0,0,0,0,0,0;0,0,0,0,0,0;dz,dz,dz,dz,dz,dz;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
        else
            O = O + [0,0,0,0,0,0;0,0,0,0,0,0;dz,dz,dz,dz,dz,dz;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
        end
        [a, b, g]= IKwithLegs(O,RG,footPositions_B,LP);
        Alpha(:,i) = transpose(a);
        Beta(:,i) = transpose(b);
        Gamma(:,i) = transpose(g);
        
    end
end

dt = 1/timing;
for i=1:(phases)*timing
    timing_sequence(i) = dt*i;
end

alpha1 = [timing_sequence(1,:);Alpha(1,:)+pi];
alpha2 = [timing_sequence(1,:);Alpha(2,:)+pi];
alpha3 = [timing_sequence(1,:);Alpha(3,:)+pi];
alpha4 = [timing_sequence(1,:);Alpha(4,:)+pi];
alpha5 = [timing_sequence(1,:);Alpha(5,:)+pi];
alpha6 = [timing_sequence(1,:);Alpha(6,:)+pi];
save alpha1 -v7.3 alpha1
save alpha2 -v7.3 alpha2
save alpha3 -v7.3 alpha3
save alpha4 -v7.3 alpha4
save alpha5 -v7.3 alpha5
save alpha6 -v7.3 alpha6

beta1 = [timing_sequence(1,:);Beta(1,:)];
beta2 = [timing_sequence(1,:);Beta(2,:)];
beta3 = [timing_sequence(1,:);Beta(3,:)];
beta4 = [timing_sequence(1,:);Beta(4,:)];
beta5 = [timing_sequence(1,:);Beta(5,:)];
beta6 = [timing_sequence(1,:);Beta(6,:)];
save beta1 -v7.3 beta1
save beta2 -v7.3 beta2
save beta3 -v7.3 beta3
save beta4 -v7.3 beta4
save beta5 -v7.3 beta5
save beta6 -v7.3 beta6

gamma1 = [timing_sequence(1,:);-pi/2 + Gamma(1,:)];
gamma2 = [timing_sequence(1,:);-pi/2 + Gamma(2,:)];
gamma3 = [timing_sequence(1,:);-pi/2 + Gamma(3,:)];
gamma4 = [timing_sequence(1,:);-pi/2 + Gamma(4,:)];
gamma5 = [timing_sequence(1,:);-pi/2 + Gamma(5,:)];
gamma6 = [timing_sequence(1,:);-pi/2 + Gamma(6,:)];
save gamma1 -v7.3 gamma1
save gamma2 -v7.3 gamma2
save gamma3 -v7.3 gamma3
save gamma4 -v7.3 gamma4
save gamma5 -v7.3 gamma5
save gamma6 -v7.3 gamma6