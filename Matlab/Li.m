% P is a orientation vector that contains the XYZ coords and ABC rotations
% of the upper platform:: Vector is 6X1

%RobotGeomtery is a data vector that contains the essental mesurements of
%the robot:: Vector is 7X1
%Format = [upperRadius, lowerRadius, alpha, beta, L1, L2, L3] 

%ALL INPUT ANGLES SHOULD BE IN DEGREES

function  [L,l,n,s,u,R,Xp] = Li(P,RobotGeometry,footPositions,LP)
%extracting inputs
Rm= RobotGeometry(1);
Rf= RobotGeometry(2);
alpha= 60*pi/180;
beta = 60*pi/180;
Xp=P(1:3,:);
a=P(4,1)*pi/180;
b=P(5,1)*pi/180;
c=P(6,1)*pi/180;
%LP = [30*pi/180, -30*pi/180, 90*pi/180, -90*pi/180, 150*pi/180, -150*pi/180];
%calulating R
R1=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)];
R2=[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)];
R3=[cos(c),-sin(c),0;sin(c),cos(c),0;0,0,1];
R=R1*R2*R3;

%calculating S
s=[Rm*cos(LP(1)),Rm*cos(LP(2)),Rm*cos(LP(3)),Rm*cos(LP(4)),Rm*cos(LP(5)),Rm*cos(LP(6));
    Rm*sin(LP(1)),Rm*sin(LP(2)),Rm*sin(LP(3)),Rm*sin(LP(4)),Rm*sin(LP(5)),Rm*sin(LP(6));
    0,0,0,0,0,0];

%calculating U
u=footPositions;%[Rf*cos(pi/2 + alpha/2),Rf*cos(alpha),Rf*cos(pi),Rf*cos(0),Rf*cos(pi+alpha),Rf*cos(-alpha);
  %  Rf*sin(pi/2 + alpha/2),Rf*sin(alpha),Rf*sin(pi),Rf*sin(0),Rf*sin(pi+alpha),Rf*sin(-alpha);
  %  .1,.1,.1,.1,0,0];

%calculating L, l, and n
for i=1:6
    L(:,i)=R*s(:,i)+Xp(:,i)-u(:,i);
    l(i)=norm(L(:,i));
    n(:,i)=L(:,i)/l(i);
end
%L is the vector of each pod and l is a vector containing the lenghths of
%pods