%% Vehicle Nonlinear Variables
% This file was derived from the work by Peter Corke and Fabian Reither.
% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
%
% http://www.petercorke.com

% Copyright 2013-2017 The MathWorks, Inc.
%For definitions: Robotic Toolbox Page 382
% 6DOF
Vehicle.SixDOF.initGreenwich = 0;
Vehicle.SixDOF.quatGain = 1;
% Position on Earth
Vehicle.PositionOnEarth.href = -init.posLLA(3);
Vehicle.PositionOnEarth.FlatEarthToLLA.xAxis = 0;
%% Physical properties
% Airframe
switch (model)
    case 'Mambo'
        Vehicle.Airframe.mass = 1.5;
        Vehicle.Airframe.inertia = diag([0.018 0.018 0.026]);
    case 'RollingSpider'
        Vehicle.Airframe.mass = 0.068;
        Vehicle.Airframe.inertia = diag([0.0686e-3 0.092e-3 0.1366e-3]);
end
Vehicle.Airframe.d = 0.22;
Vehicle.Airframe.xy = Vehicle.Airframe.d*sqrt(2)/2; % For diamond rotor set-up
Vehicle.Airframe.h = -0.1;
Vehicle.Airframe.Cdx = 0;
Vehicle.Airframe.Cdy = 0;
Vehicle.Airframe.diameter = 0.22; % For drag calculation purposes
% Rotor
Vehicle.Rotor.blades = 2;
Vehicle.Rotor.radius = 0.11;
Vehicle.Rotor.chord = .008;
Vehicle.Rotor.flappingOffset = 0;
Vehicle.Rotor.bladeMass = 5e-3;
Vehicle.Rotor.bladeInertia = Vehicle.Rotor.bladeMass*Vehicle.Rotor.radius^2/4;
Vehicle.Rotor.hubMass = 0;
Vehicle.Rotor.hubInertia = 0;
Vehicle.Rotor.inertia = Vehicle.Rotor.hubInertia + Vehicle.Rotor.bladeInertia;
Vehicle.Rotor.Ct = .0107;
Vehicle.Rotor.Cq = Vehicle.Rotor.Ct*sqrt(Vehicle.Rotor.Ct/2);
Vehicle.Rotor.solidity = Vehicle.Rotor.chord*Vehicle.Rotor.blades/(pi*Vehicle.Rotor.radius);
Vehicle.Rotor.theta0 = 14.6*(pi/180);
Vehicle.Rotor.thetaTip = 6.8*(pi/180);
Vehicle.Rotor.theta1 = Vehicle.Rotor.thetaTip-Vehicle.Rotor.theta0;
Vehicle.Rotor.theta34 = Vehicle.Rotor.theta0+0.75*Vehicle.Rotor.theta1;
Vehicle.Rotor.a = 5.5; % Lift slope
Vehicle.Rotor.area = pi*Vehicle.Rotor.radius^2;
Vehicle.Rotor.lock = rho*Vehicle.Rotor.a*Vehicle.Rotor.chord*Vehicle.Rotor.radius^4/...
    (Vehicle.Rotor.hubInertia+Vehicle.Rotor.bladeInertia);
Vehicle.Rotor.b = Vehicle.Rotor.Ct*rho*Vehicle.Rotor.area*Vehicle.Rotor.radius^2;
Vehicle.Rotor.k = Vehicle.Rotor.Cq*rho*Vehicle.Rotor.area*Vehicle.Rotor.radius^3;
Vehicle.Rotor.w2ToThrustGain = Vehicle.Rotor.Ct*rho*Vehicle.Rotor.area*Vehicle.Rotor.radius^2;
% Motors
Vehicle.Motor.maxLimit = 500;
Vehicle.Motor.minLimit = 10;
Vehicle.Motor.commandToW2Gain = 4066.09; %motor command for Rolling Spider (0-500) to motorspeed^2
Vehicle.Motor.thrustToMotorCommand = 1/(Vehicle.Rotor.w2ToThrustGain*Vehicle.Motor.commandToW2Gain);