function QuadCopter = QuadCopterModel
[X1,Y1,Z1] = Extrude(@(t) shape1(t), {@(t) pathx1(t), @(t) pathy1(t), @(t) pathz1(t)}, [0,0.1,0.2,0.8,0.9,1], [25,5,1,5,25]);
[X2,Y2,Z2] = Extrude(@(t) shape2(t), {@(t) -3-5*t, @(t) -2.5-5*t, @(t) 0*t}, [0, 1], 2); 
[X3,Y3,Z3] = cylinder(0.5,20); X3 = X3 + 8; Y3 = Y3 + 7.5; Z3 = 2*Z3 - 1; 
[X, Y, Z] = cylinder([0, 0.6,0.6,0.6,0.6],20);
X3a = X + 8; Y3a = Y + 7.5; Z3a = 0.5*Z - 1.5;
[X, Y, Z] = cylinder([0.6,0.6,0.6,0.6,0],20); 
X3b = X + 8; Y3b = Y + 7.5; Z3b = 0.5*Z + 1;
X3 = [X3a;X3;X3b];  Y3 = [Y3a;Y3;Y3b]; Z3 = [Z3a;Z3;Z3b];   

[X, Y, Z] = cylinder([0.1, 0.05, 0.05, 0.05],20);
X4 = X + 8; Y4 = Y + 7.5; Z4 = 0.2*Z + 1.5;

[X,Y,Z] = tubeN(@(t) 0.1+0*t,20,{@(t) 8+4*cos(0)*cos(t), @(t) 7.5+4*sin(0)*cos(t), @(t) 3*sin(t)}, [pi/2, 0], 20); 
X5 = [flipud(X);X]; Y5 = [flipud(Y);Y]; Z5 = [flipud(Z);-Z+6];
[X,Y,Z] = tubeN(@(t) 0.1+0*t,20,{@(t) 8+4*cos(pi/2)*cos(t), @(t) 7.5+4*sin(pi/2)*cos(t), @(t) 3*sin(t)}, [pi/2, 0], 20); 
X6 = [flipud(X);X]; Y6 = [flipud(Y);Y]; Z6 = [flipud(Z);-Z+6];
[X7,Y7,Z7] = Extrude(@(t) shape7(t), {@(t) 8+4*cos(t), @(t) 7.5+4*sin(t), @(t) 0*t}, [5*pi/6, -pi/3], 20); 

%rotors
[X,Y,Z] = Extrude(@(t) shape8(t), {@(t) 3*t, @(t) 0*t, @(t) 1.59+0*t}, [0,0.1,0.2,1.2], [1,10,1]); 
X = [flipud(X);-X]; Y = [flipud(Y);-Y]; Z = [flipud(Z);Z]; [s1,s2] = size(X);
V = 1/sqrt(2)*[X(:),Y(:)]*[1, 1;-1,1];
X8 = reshape(V(:,1), s1, s2)+8; Y8 = reshape(V(:,2), s1, s2)+7.5; Z8 = Z;

figure('Units', 'pixels','Position', [1 41 1220 763], 'Color', 'w')
hold on; axis equal;  lighting gouraud; camlight;
I = eye(3); %Identity matrix 3D;

Body.GPS   = [0,0,0];  %% Center of Gravity of the Helicopter
Body.GYRO  = [0,0,0]; %% be replaced with get GYRO orientation
Body.Axis  = I; %% be replaced with get AXIS orientation

Body.Model = {surf(X1,Y1,Z1+4,'edgealpha', 0.3, 'facecolor', 'w'); 
              surf(X2,Y2,Z2+4,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'r');
              surf(-X2,Y2,Z2+4,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'r');
              surf(-X2,-Y2,Z2+4,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'r');
              surf(X2,-Y2,Z2+4,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'r');
              surf(X3,Y3,Z3+4,'edgealpha', 0.3, 'facecolor', 'g');
              surf(X3,Y3-15,Z3+4,'edgealpha', 0.3, 'facecolor', 'g');
              surf(X3-16,Y3-15,Z3+4,'edgealpha', 0.3, 'facecolor', 'g');
              surf(X3-16,Y3,Z3+4,'edgealpha', 0.3, 'facecolor', 'g');
              surf(X4,Y4,Z4+4,'edgealpha', 0.3, 'facecolor', 'y');
              surf(X4,Y4-15,Z4+4,'edgealpha', 0.3, 'facecolor', 'y');
              surf(X4-16,Y4-15,Z4+4,'edgealpha', 0.3, 'facecolor', 'y');
              surf(X4-16,Y4,Z4+4,'edgealpha', 0.3, 'facecolor', 'y');
              surf(X5,Y5,Z5,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(-X5,Y5,Z5,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(-X5,-Y5,Z5,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(X5,-Y5,Z5,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(X6,Y6,Z6,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(-X6,Y6,Z6,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(-X6,-Y6,Z6,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(X6,-Y6,Z6,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(X7,Y7,Z7,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(-X7,Y7,Z7,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(-X7,-Y7,Z7,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(X7,-Y7,Z7,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(X7,Y7,Z7+6,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(-X7,Y7,Z7+6,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(-X7,-Y7,Z7+6,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b');
              surf(X7,-Y7,Z7+6,'edgealpha', 0.3, 'facealpha', 0.7, 'facecolor', 'b')};         
          
Rotors.GPS   = [8,-7.5,1.58
                8,7.5,1.58
                -8,7.5,1.58
                -8,-7.5,158];  %% be replaced with get GPS location
Rotors.GYRO  = [0,0,0]; %% be replaced with get GYRO orientation
Rotors.Axis  = I; %% be replaced with get AXIS orientation
Rotors.Model = { surf(X8,Y8-15,Z8+4,'edgealpha', 0.3, 'facecolor', 'y');
                 surf(X8,Y8,Z8+4,'edgealpha', 0.3, 'facecolor', 'y');
                 surf(X8-16,Y8,Z8+4,'edgealpha', 0.3, 'facecolor', 'y');
                 surf(X8-16,Y8-15,Z8+4,'edgealpha', 0.3, 'facecolor', 'y')}; hold off;
             
State           = [Body.GPS';zeros(9,1)];
RotorsSpeed     = zeros(4,1);
TimeStep        = 0;
l               = 0.15;
k               = 1.980e-5;
b               = 2.140e-6;
MomIn           = [7.856e-3;7.856e-3;1.4702e-2];
m               = 1.768;

%% Operators
QuadCopter.ArmLength       = l;
QuadCopter.LiftCoefficient = k;
QuadCopter.RotorDrag       = b;
QuadCopter.Axis            = Body.Axis;
QuadCopter.State           = State;
QuadCopter.Mass            = m;
QuadCopter.MomIn           = MomIn;
QuadCopter.UpdateState     = @(delState) StateUpdate(delState);
QuadCopter.Dynamics        = @(Omegas, dt) Dynamics(Omegas, dt);
QuadCopter.AirResistance   = zeros(3);
QuadCopter.wind            = 0*rand(3,1);

    function TranslateQuadcopter3D(dr)
        Body.GPS = Body.GPS + dr; 
        for n = 1:numel(Body.Model)
            Body.Model{n}.XData = Body.Model{n}.XData + dr(1);
            Body.Model{n}.YData = Body.Model{n}.YData + dr(2);
            Body.Model{n}.ZData = Body.Model{n}.ZData + dr(3);
        end

        Rotors.GPS = Rotors.GPS + ones(4,1)*dr; 
        for n = 1:numel(Rotors.Model)
            Rotors.Model{n}.XData = Rotors.Model{n}.XData + dr(1);
            Rotors.Model{n}.YData = Rotors.Model{n}.YData + dr(2);
            Rotors.Model{n}.ZData = Rotors.Model{n}.ZData + dr(3);
        end
    end
    function RotateQuadcopter3D(delAngles)
        Body.GYRO = Body.GYRO + delAngles; % Updating GYRO
        Matrix  = Rzyx(delAngles);
        Body.Axis = Matrix*Body.Axis; %Updating Axis
        for n = 1:numel(Body.Model)
            [s1,s2] = size(Body.Model{n}.XData);
            V = [Body.Model{n}.XData(:) - Body.GPS(1), ...
                 Body.Model{n}.YData(:) - Body.GPS(2),...
                 Body.Model{n}.ZData(:) - Body.GPS(3)];
            V = V*Matrix';
            Body.Model{n}.XData = reshape(V(:,1) + Body.GPS(1), s1, s2);
            Body.Model{n}.YData = reshape(V(:,2) + Body.GPS(2), s1, s2);
            Body.Model{n}.ZData = reshape(V(:,3) + Body.GPS(3), s1, s2);
        end
        
        Rotors.GPS = (Rotors.GPS - ones(4,1)*Body.GPS)*Matrix' + ones(4,1)*Body.GPS;
        Rotors.Axis = Matrix*Rotors.Axis; %Updating Axis
        for n = 1:numel(Rotors.Model)
            [s1,s2] = size(Rotors.Model{n}.XData);
            V = [Rotors.Model{n}.XData(:) - Body.GPS(1), ...
                 Rotors.Model{n}.YData(:) - Body.GPS(2),...
                 Rotors.Model{n}.ZData(:) - Body.GPS(3)];
            V = V*Matrix';
            Rotors.Model{n}.XData = reshape(V(:,1) + Body.GPS(1), s1, s2);
            Rotors.Model{n}.YData = reshape(V(:,2) + Body.GPS(2), s1, s2);
            Rotors.Model{n}.ZData = reshape(V(:,3) + Body.GPS(3), s1, s2);
        end
    end
    function RotorsRotate
        for n = 1:numel(Rotors.Model)
            Matrix = Mxyz(Rotors.Axis(:,3),(-1)^(n)*RotorsSpeed(n)*TimeStep*2*pi/60)';
            [s1,s2] = size(Rotors.Model{n}.XData);
            V = [Rotors.Model{n}.XData(:) - Rotors.GPS(n,1), ...
                 Rotors.Model{n}.YData(:) - Rotors.GPS(n,2),...
                 Rotors.Model{n}.ZData(:) - Rotors.GPS(n,3), ones(s1*s2,1)];
            V = V*Matrix';
            Rotors.Model{n}.XData = reshape(V(:,1) + Rotors.GPS(n,1), s1, s2);
            Rotors.Model{n}.YData = reshape(V(:,2) + Rotors.GPS(n,2), s1, s2);
            Rotors.Model{n}.ZData = reshape(V(:,3) + Rotors.GPS(n,3), s1, s2);
        end
    end  

    function ds = Dynamics(wsqr, dt)
        TimeStep = dt; RotorsSpeed = sqrt(wsqr); Axis = Body.Axis; s = State;
        Ixx   = MomIn(1) ;  Iyy = MomIn(2);  Izz = MomIn(3);  g = 9.81;
        A = QuadCopter.AirResistance; wind = QuadCopter.wind;
        factors = [k, k*l, k*l, b];
        Ft    = factors(1)*(wsqr(1) + wsqr(2) + wsqr(3) + wsqr(4)); 
        taux  = factors(2)*(wsqr(1) - wsqr(2) - wsqr(3) + wsqr(4));
        tauy  = factors(3)*(wsqr(1) + wsqr(2) - wsqr(3) - wsqr(4));
        tauz  = factors(4)*(wsqr(1) - wsqr(2) + wsqr(3) - wsqr(4));
        function ds = dsdt(s)
            posp  = s(7:9); etap  = s(10:12);
            p     = s(10);    q      = s(11);      r     = s(12); 
            pospp = [0;0;-g] + (Ft/m)*Axis(:,3) + (A*(0*wind - posp))/m;
            etapp = [((Iyy - Izz)*(q*r) + taux)/Ixx
                     ((Izz - Ixx)*(p*r) + tauy)/Iyy
                     ((Ixx - Iyy)*(p*q) + tauz)/Izz];
            ds = [posp;etap;pospp;etapp];
        end
        K1 = dt*dsdt(s); K2 = dt*dsdt(s + 0.5*K1);
        K3 = dt*dsdt(s + 0.5*K2); K4 = dt*dsdt(s + K3);
        ds = (K1 + 2*K2 + 2*K3 + K4)/6;
    end
    function quadcopter = StateUpdate(delState)
        TranslateQuadcopter3D(delState(1:3)');
        RotateQuadcopter3D(delState(4:6)');
        RotorsRotate;
        State = State + delState;
        quadcopter = QuadCopter;
        quadcopter.State = State;
        quadcopter.Axis = Body.Axis;
    end
end

%% Rotation Matrix and Axis
function M = Mxyz(V, theta)
    M = eye(4); x = V(1); y = V(2); z = V(3); 
    c = cos(theta); s = sin(theta); I = 1:3; 
    M(I,I) = [x*x*(1-c)+  c, y*x*(1-c)-z*s, z*x*(1-c)+y*s
              y*x*(1-c)+z*s, y*y*(1-c)+  c, z*y*(1-c)-x*s
              z*x*(1-c)-y*s, y*z*(1-c)+x*s, z*z*(1-c)+  c];
end
      
function R = Rzyx(GYRO)
    phi    = GYRO(1); Cphi   = cos(phi);   Sphi   = sin(phi);    
    theta  = GYRO(2); Ctheta = cos(theta); Stheta = sin(theta);  
    xi     = GYRO(3); Cxi    = cos(xi);    Sxi    = sin(xi); 
    R      = [Cxi*Ctheta, Cxi*Stheta*Sphi - Sxi*Cphi, Cxi*Stheta*Cphi + Sxi*Sphi
              Sxi*Ctheta, Sxi*Stheta*Sphi + Cxi*Cphi, Sxi*Stheta*Cphi - Cxi*Sphi
                -Stheta ,          Ctheta*Sphi      ,         Ctheta*Cphi       ];  
end

function [Y,Z] = shape1(TT)
    t1 = linspace(pi,0,15); c1 = cos(t1); s1 = sin(t1); 
    sc1 = sign(c1); ss1 = sign(s1); ac1 = abs(c1); as1 = abs(s1);
    t2 = linspace(0,-pi,15); c2 = cos(t2); s2 = sin(t2); 
    sc2 = sign(c2); ss2 = sign(s2); ac2 = abs(c2); as2 = abs(s2);
    Y = []; Z = [];
    for n = 1:numel(TT)
        tt = TT(n);
        if(tt < 0.1)
            t = sqrt(10*tt); y = t*2*c1; z = 0.6 + t*0.4*s1;
            y = [y,t*2*c2,y(1)]; z = [z, 0.6 + t*0.4*s2,z(1)];
            Y = [Y;y]; Z = [Z;z];
        elseif(tt < 0.2)
            t = 10*(tt-0.1); y1 = 2*c1; z1 = 0.6 + 0.4*s1;
            y1 = [y1, 2*c2, y1(1)]; z1 = [z1, 0.6 + 0.4*s2, z1(1)];

            y2 = 3*c1; z2 = ss1.*sqrt(as1);
            y2 = [y2,3*c2,y2(1)]; z2 = [z2,ss2.*sqrt(as2),z2(1)];

            t = 0.5*(tanh(5*(t - 0.5)) + 1);
            y = y1 + t*(y2 - y1); z = z1 + t*(z2 - z1);
            Y = [Y;y]; Z = [Z;z];
        elseif(tt < 0.8)
            y = 3*c1; z = ss1.*sqrt(as1);
            y = [y,3*c2,y(1)]; z = [z,ss2.*sqrt(as2),z(1)];
            Y = [Y;y]; Z = [Z;z];
        elseif(tt < 0.9)
            t = 10*(tt-0.8);
            y1 = 3*c1; z1 = ss1.*sqrt(as1);
            y1 = [y1,3*c2,y1(1)]; z1 = [z1,ss2.*sqrt(as2),z1(1)];

            y2 = 2*c1; z2 = 0.6 + 0.4*s1;
            y2 = [y2, 2*c2, y2(1)]; z2 = [z2, 0.6 + 0.4*s2, z2(1)];

            t = 0.5*(tanh(5*(t - 0.5)) + 1);
            y = y1 + t*(y2 - y1); z = z1 + t*(z2 - z1);
            Y = [Y;y]; Z = [Z;z];
        else
            t = sqrt(10*(1 - tt)); y = t*2*c1; z = 0.6 + t*0.4*s1;
            y = [y,t*2*c2,y(1)]; z = [z, 0.6 + t*0.4*s2,z(1)];
            Y = [Y;y]; Z = [Z;z];
        end
    end
end

function x = pathx1(t)
    x = 10*t - 5;
    x(t<0.1) = 40*t(t<0.1)-8;
    x(t>0.9) = 40*t(t>0.9)-32;
end

function y = pathy1(t)
    y = 0*t;
end

function z = pathz1(t)
    z = 0*t;
end

function [Y,Z] = shape2(TT)
    t1 = linspace(pi,0,15); c1 = cos(t1); s1 = sin(t1); ss1 = sign(s1); as1 = abs(s1);
    t2 = linspace(0,-pi,15); c2 = cos(t2); s2 = sin(t2); ss2 = sign(s2);as2 = abs(s2);
    y = 0.3*c1; z = 0.3+0.1*ss1.*sqrt(as1);
    y = [y,0.3*c2,y(1)]; z = [z,-0.3+0.1*ss2.*sqrt(as2),z(1)];
    Y = ones(size(TT))*y; Z = ones(size(TT))*z;
end

function [Y,Z] = shape7(TT)
    t1 = linspace(pi,0,5); c1 = cos(t1); s1 = sin(t1); ss1 = sign(s1); as1 = abs(s1);
    t2 = linspace(0,-pi,5); c2 = cos(t2); s2 = sin(t2); ss2 = sign(s2);as2 = abs(s2);
    y = 0.1*c1; z = 0.25+0.05*ss1.*sqrt(as1);
    y = [y,0.1*c2,y(1)]; z = [z,-0.25+0.05*ss2.*sqrt(as2),z(1)];
    Y = ones(size(TT))*y; Z = ones(size(TT))*z;
end

function [Y,Z] = shape8(TT)
    t = linspace(0,2*pi,30); c = cos(t); ac = abs(c); s = sin(t); as = abs(s);
    B = 2; T = 0.2; C = 0.05; P = 1; E = 1; R = 0; Y = []; Z = [];
    y1 = 0.07*c; z1 = 0.07*s; y2 = 0.5+0.5*ac.^B./c;
    z2 = T/2*as.^B./s.*(1-y2.^P) + C.*sin(y2.^E*pi) + R*sin(2*pi*y2)-0.035;
    y2 = y2 - 0.13;
    for n = 1:numel(TT)
        tt = TT(n);
        if(tt < 0.1)
            Y = [Y;y1]; Z = [Z;z1];
        elseif(tt<0.2)
            t = 10*(tt-0.1);
            t = 0.5*(tanh(5*(t - 0.5)) + 1);
            Y = [Y;y1+t*(0.5*y2 - y1)]; Z = [Z;z1+t*(0.5*z2 - z1)];
        else
            Y = [Y;0.5*y2]; Z = [Z;0.5*z2];
        end
    end
end