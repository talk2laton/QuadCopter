function FlightController(QuadCopter, TargetList, Tol, videoname)
clc;
global vidObj
writevideo = exist('videoname','var');
QuadCopter.AirResistance = diag(0.25*ones(3,1));
framerate = 11; view(-37.5,30); t = 0; %asinaz = 0;
%% Start count down
CountDownFrom(5);

%% Capturing Video
if(writevideo)
    vidObj = VideoWriter(videoname); set(vidObj, 'FrameRate',framerate); open(vidObj);
end
Totaltime = 0;
%% Navigating
for n = 1:size(TargetList,1)
    Target = TargetList(n,:)'; error = norm(Target - QuadCopter.State(1:3)) ;
    iter = 0;Ierror = zeros(12,1); 
    if n == size(TargetList,1)
        Tol = Tol/10;
    end
    while (error > Tol && iter < 5000)
        dt = 1/framerate;
        Omegas2 = PDController(QuadCopter, Target, Ierror);
        dstate = QuadCopter.Dynamics(Omegas2, dt); t = t + dt;
        QuadCopter = QuadCopter.UpdateState(dstate);
        title(['going to point ',num2str(n),' , at speed of ', num2str(norm(QuadCopter.State(7:9))),'m/s']);
        drawnow; iter = iter + 1; %asinaz = asinaz + 0.0005; view(-37.5+90*sin(asinaz),30);
%         Ierror = Ierror + ([Target;zeros(9,1)] - QuadCopter.State)*dt;
        error = norm(Target - QuadCopter.State(1:3));
        if(writevideo)
            image = getframe(gcf);
            writeVideo(vidObj,image);
        end
    end
    
    %% Hover mode
    disp('===============================================================');
    Totaltime = Totaltime + iter/framerate; disp(Totaltime);
    disp('===============================================================');
%     for i = 1:200
%         Omegas2 = PDController(QuadCopter, Target, Ierror);
%         dstate = QuadCopter.Dynamics(Omegas2, dt); t = t + dt;
%         QuadCopter = QuadCopter.UpdateState(dstate); drawnow;
%         title(['at point ',num2str(n)]);
%         if(writevideo)
%             image = getframe(gcf);
%             writeVideo(vidObj,image);
%         end
%     end
end
if(writevideo)
    close(vidObj);
end

function CountDownFrom(N)
i = N;
for n = 1:N
    pause(1);
    title(num2str(i));
    i = i - 1;
end
pause(1);
title('Start');

function omegas2 = PDController(QuadCopter, Target, Ierr)
Ix = QuadCopter.MomIn(1); Iy = QuadCopter.MomIn(2); 
Iz = QuadCopter.MomIn(3); l = QuadCopter.ArmLength; 
k = QuadCopter.LiftCoefficient;b = QuadCopter.RotorDrag; 
Axis = QuadCopter.Axis; s = QuadCopter.State;
m = QuadCopter.Mass;
K = [0.0063    0.0248    0.0248
     0.0107    0.0742    0.0742
     2.6151    7.8633    7.8633
     1.9020    2.3727    2.3727
     1.8608    2.0378    2.0378
     0.0815    6.1876    6.1876]; %PDCOntrollerGain
P = K(:,1);  I = K(:,2); D = K(:,3); g = 9.81; 
theta = (pi/6)*max(-1, min(1,(P(1)*(Target(1) - s(1)) + I(1)*Ierr(1) - D(1)*s(7))));
phi   = (pi/6)*max(-1, min(1,-(P(2)*(Target(2) - s(2)) + I(2)*Ierr(2) - D(2)*s(8))));
xi    = 0;
Tzz   = P(3)*(Target(3) - s(3)) + I(3)*Ierr(3) - D(3)*s(9);
Tz    = max(0, m*(g + min(10,Tzz)))/Axis(end); %thrust
taux  = (P(4)*(phi   - s(4)) + I(4)*Ierr(4) - D(4)*s(10))*Ix; %roll
tauy  = (P(5)*(theta - s(5)) + I(5)*Ierr(5) - D(5)*s(11))*Iy; %pitch
tauz  = (P(6)*(xi    - s(6)) + I(6)*Ierr(6) - D(6)*s(12))*Iz; % yaw
% disp([Target'; s(1:3)'; del'; s(4:6)'; s(7:9)'; s(10:12)';[tauy,taux,tauz]]);
% disp('===============================================================');
omegas2 = [Tz/(4*k) + tauz/(4*b) + tauy/(4*k*l) + taux/(4*k*l);
           Tz/(4*k) - tauz/(4*b) + tauy/(4*k*l) - taux/(4*k*l);
           Tz/(4*k) + tauz/(4*b) - tauy/(4*k*l) - taux/(4*k*l);
           Tz/(4*k) - tauz/(4*b) - tauy/(4*k*l) + taux/(4*k*l)]; 
minO    = min(omegas2);
if(minO < 0)
    omegas2 = omegas2 - minO;
end
