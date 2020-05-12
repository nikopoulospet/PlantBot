% P is a orientation vector that contains the XYZ coords and ABC rotations
% of the upper platform:: Vector is 6X1

%RobotGeomtery is a data vector that contains the essental mesurements of
%the robot:: Vector is 7X1
%Format = [upperRadius, lowerRadius, alpha, beta, L1, L2, L3] 

%ALL INPUT ANGLES SHOULD BE IN DEGREES

function [Alpha, Beta, Gamma]= IKwithLegsBACKUP(P,RobotGeometry,footPositions,LP)
%IKWITHLEGS Summary of this function goes here
%   Detailed explanation goes here

Lprime = zeros(3,6);
h = zeros(1,6);
hprime = zeros(1,6);
Rho = zeros(1,6);
Phi = zeros(1,6);
S2 = zeros(3,6);
Alpha = zeros(1,6);
Beta = zeros(1,6);
Gamma = zeros(1,6);
L1 = RobotGeometry(5);
L2 = RobotGeometry(6);
L3 = RobotGeometry(7);
[L,~,~,S,U,R,Xp] = Li(P,RobotGeometry,footPositions,LP);

for i=1:6
    if (-1)^i ~= 1
        Alpha(i) = pi + atan(L(2,i)/L(1,i)); %if an odd leg add pi bcs of arctan range
        
    else
        Alpha(i) = atan(L(2,i)/L(1,i));
    end
    S2(:,i)= [S(1,i) + L1*cos(Alpha(i)); ...
        S(2,i) + L1*sin(Alpha(i)); ...
        S(3,i)];
    Lprime(:,i)=Xp(:,i)+R*S2(:,i)-U(:,i);
    h(i) = Lprime(3,i);
    hprime(i) = L(3,i); 
    Phi(i) = asin((hprime(i)-h(i))/L1);
    Rho(i) = atan(hprime(i)/sqrt((Lprime(1,i)^2)+(Lprime(2,i)^2)));
    Beta(i) = acos((L2^2+norm(Lprime(:,i))^2-L3^2)/(2*L2*norm(Lprime(:,i)))) - (Phi(i) + Rho(i));
    Gamma(i) = pi-acos((L2^2-norm(Lprime(:,i))^2+L3^2)/(2*L2*L3));
end
end


