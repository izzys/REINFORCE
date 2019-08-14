global   simulationTimeStep targetSpeed maxTurnRate H FOV switchTimeConstant rateCommandLimit
global Wn zetta frameRate Tf
d2r=pi/180; ft2m=.3048;
simulationTimeStep=0.005; % sec
targetSpeed=50*1000/3600;  % m/sec
maxTurnRate=30*d2r; % rad/sec
FOV=5*d2r; % rad
H=10000*ft2m; % m
switchTimeConstant=(H*FOV/targetSpeed)/5; 
rateCommandLimit=5*d2r;
Wn=10*2*pi; zetta=0.9;
frameRate=1/simulationTimeStep; 
Tf=5; 


