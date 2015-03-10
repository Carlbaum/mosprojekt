function copter3()
    clear all
    % konstanter
    g = 9.82;       % gravitationskonstant
    m = 0.5;          % massa
    L = 0.25;       % l?ngd fr?n kroppens centrum till mitten av en propeller;
    k = 3e-6;   % propellerkonstant, inkl. lufttryck etc.
    b = 1e-7;          % drag constant
    I = diag([1 1 2].*0.005); % tr?ghetsmoment f?r copterns vridning
%     IMR = 1;                  % rotorernas tr?ghetsmoment
    

    % initialvillkor
    eta      = [ 0; pi/11 ; 0 ]; % vinklar; roll, pitch, yaw
    phiDot   = 0; % vinkelhastighet
    thetaDot = 0;
    psiDot   = 0;
    etaDot = [phiDot;thetaDot;psiDot];
%     etaDotDot = zeros(3,1);
%         % body frame rotations
%         bAngVel = zeros(3,1); 
%         bAngAcc = zeros(3,1);

    input = zeros(4,1); % vinkelhastigheter p? varje rotor
    
%     thrust = [0;0;0];         % agerar bara i kroppens lokala och positiva z-led
    
    disturbanceForces = [0.0;0.0;0.0];
    
    pos = [0;0;10]; % globala koord. systemet
    vel = [0;0;0];
    acc = acceleration(g,m,eta,disturbanceForces,0);

    % referenser
    refPos = [  0;  0; 16];   % Dessa ska kunna ?ndras av anv?ndaren
    refEta = [  0;  0;  0];
    phiC   = 0;
    thetaC = 0;
    psiRef = 0;             % Denna ska kunna ?ndras av anv?ndaren
    
    % Variables for PID-kontroller
    errPos   = refPos - pos;
    errEta = refEta - eta;
%     err = [errPos; errEta];
    
%     integral = 0;
%     Kmain = 1; % dimensionsl?s om e och resultatet av PID:en, u, ?r i samma enhet 
%     Ti = 1; % tid i sekunder 
%     Td = 1; % tid i sekunder 
    
%     KPosP  = 0.85;
%     KPosD  = 0.75;
%     KPosDD = 0.5;
%     KAngP  = 4;
%     KAngD  = 0.50;
KPosP = [ 4.0 ; 0.0 ; 4.0 ];
KPosD = [ 5.50 ; 0 ; 5.50 ];
KPosDD =[ 0.75 ; 0; 0.75 ];
KAngP = 12.5;
KAngPY = 12.5;
KAngD = 1.50;
KAngDY = 1.50;
    KAngDD = 0.0;%5;
    % tidsspann, sekunder
    tStart = 0;
    tStop = 50;
    tStep = 0.001;
    time = tStart:tStep:tStop;
    totalSteps = numel(time);

    % vektorer f?r att lagra v?rden f?r plottar
    posStored = [ pos zeros(3, totalSteps-1)];
    velStored = [ vel zeros(3, totalSteps-1)];
    accStored = [ acc zeros(3, totalSteps-1)];

    etaStored = [ eta zeros(3, totalSteps-1)];

    inputStored = input;
   
    counter = 1;
    phiDotC =0;
    thetaDotC=0;
    c=0;
    etaDotDot = [0;0;0];
    psiDotRef = 0;
    for t = time(2:end) % f?rsta tidssteget behandlas redan innan loopen
        counter = counter + 1;
        
        % Ber?kna avvikelser fr?n v?rat ?nskade state
        errPos = refPos - pos;
        errVel = -vel; % it's really  refVel-vel.. but refVel = [0,0,0];
        errAcc = -acc; % it's really  refAcc-acc.. but refAcc = [0,0,0];
        
        dvec = KPosP.*errPos + KPosD.*errVel + KPosDD.*errAcc;
        dx = dvec(1);
        dy = dvec(2);
        dz = dvec(3);
        
        phi   = eta(1); % roll
        theta = eta(2); % pitch
        psi   = eta(3); % yaw
        
        W = [ 1, 0, -sin(theta) ;
              0, cos(phi), cos(theta)*sin(phi);
              0, -sin(phi), cos(theta)*cos(phi)];
       
        phiCPrev   = phiC;
        thetaCPrev = thetaC;
        psiRPrev   = psiRef;
        
        % Commanded phi & theta.. 
        phiC = asin((dx*sin(psi) - dy*cos(psi))...
            /(dx^2 + dy^2 + (dz+g)^2));
        thetaC = atan((dx*cos(psi) + dy*sin(psi))...
            /(dz+g));
        
        phiDCPrev = phiDotC;
        thetaDCPrev = thetaDotC;
       % psiDRefPrev = psiDotRef;        
        
        phiDotC    = ( phiCPrev - phiC ) /tStep;
        thetaDotC  = (thetaCPrev-thetaC) /tStep;
       % psiDotRef  = ( psiRPrev -psiRef) /tStep;
        
        phiDot   = etaDot(1);
        thetaDot = etaDot(2);
        psiDot   = etaDot(3);
        
        phiDDC = ( phiDCPrev - phiDotC )/tStep;
        thetaDDC = ( thetaDCPrev - thetaDotC )/tStep;
        %psiDDC = ( psiDCPrev - psiDotC ) / tStep;
        phiDD = etaDotDot(1);
        thetaDD = etaDotDot(2);
        
        totalThrust = m*(dx*( sin(theta)*cos(psi)*cos(phi) + sin(psi)*sin(phi) )...
                        +dy*( sin(theta)*sin(psi)*cos(phi) - cos(psi)*sin(phi) )...
                        +(dz+g)*cos(theta)*cos(phi));
        if totalThrust < 0
            c = c+1;
             totalThrust = 0;
        end
                    
        torquePhi   = ( KAngP *(phiC - phi)   + KAngD *(phiDotC  - phiDot ) )*I(1,1);
        torqueTheta = ( KAngPY*(thetaC-theta) + KAngDY*(thetaDotC-thetaDot) )*I(2,2);
        torquePsi   = ( KAngP *(psiRef- psi ) + KAngD *(psiDotRef- psiDot ) )*I(3,3);
        tau = [torquePhi;torqueTheta;torquePsi];
        
        % Calculate inputs(each rotor's angular velocity (angVel)).. 
        % it's actually angVel^2.. but it makes no difference in our case..
        input(1) = totalThrust/(4*k) - torqueTheta/(2*k*L) - torquePsi/(4*b);
        input(3) = totalThrust/(4*k) + torqueTheta/(2*k*L) - torquePsi/(4*b); 
        input(2) = totalThrust/(4*k) - torquePhi  /(2*k*L) + torquePsi/(4*b); 
        input(4) = totalThrust/(4*k) + torquePhi  /(2*k*L) + torquePsi/(4*b);
%         temp = input > 15/k/4;
%         input(temp) = 15/k/4;
        
        % Angular velocity and acceleration in the body frame
        bAngVel = W*etaDot;
        bAngAcc = I\(tau - cross(bAngVel, I*bAngVel));       
        
        % Uppdatera positioner och vinklar i systemet
        % Uppdatera vinklar
        bAngVel   = bAngVel + bAngAcc*tStep;
        etaDot    = W\bAngVel;
        eta       = eta + etaDot*tStep;
        
        etaDotDot = [0, etaDot(1)*cos(phi)*tan(theta)+etaDot(2)*sin(phi)/(cos(theta)^2), -etaDot(1)*sin(phi)*cos(theta)+etaDot(2)*cos(phi)/(cos(theta)^2);...
             0, -etaDot(1)*sin(phi), -etaDot(1)*cos(phi);
             0, (etaDot(1)/cos(theta))*(cos(phi)+sin(phi)*tan(theta)),(-etaDot(1)*sin(phi)+etaDot(2)*cos(phi)*tan(theta))/cos(theta)]...
             *bAngVel + W\bAngAcc; 
                
        % Ber?kna acceleration, hastighet och position
        acc = acceleration(g,m,eta,disturbanceForces,totalThrust);
        vel = vel + acc*tStep;
        pos = pos + vel*tStep;
        
        %store values for plots
        inputStored(:,counter) = input;
        etaStored(:,counter) = radtodeg(eta);
        accStored(:,counter) = acc;
        velStored(:,counter) = vel;
        posStored(:,counter) = pos;
    end
    subplotFunc(time,accStored,'Acceleration',velStored,'Velocity',posStored,'Position',sprintf('Kp = %0.2f, Kd = %0.2f, Kdd = %0.2f',KPosP,KPosD,KPosDD));
    figure
    plot(time,etaStored(1,:),'r');
    hold on
    plot(time,etaStored(2,:),'g');
    plot(time,etaStored(3,:),'b');
    movegui('northeast') 
    title('Angles')
    legend('roll','pitch','yaw')
    titleString = sprintf('Kp = %0.2f,  Kd = %0.2f',KAngPY,KAngDY);
    ha = axes('Position',[0 0.9 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
    text(0.5, 0.1,titleString,'HorizontalAlignment' ,'center','VerticalAlignment', 'top');
    
    figure
     plot(time,inputStored(1,:),'r');
     hold on
     plot(time,inputStored(2,:),'g');
     plot(time,inputStored(3,:),'b');
     plot(time,inputStored(3,:),'m');
     movegui('south') 
     title('Inputs, angular velocities of each rotor')
     legend('Front rotor','Right rotor','Back rotor','Left rotor')
end
function acc = acceleration(g,m,eta,disturbanceForces,T)
    acc = [0;0;-g] + (rotation(eta)*[0;0;T])./m + disturbanceForces./m; 
end
function rotMat = rotation( eta )
    roll    = eta(1);
    pitch   = eta(2);
    yaw     = eta(3);

    rotMat = zeros(3);
    rotMat(:, 1) = [
        cos(yaw) * cos(pitch)
        cos(pitch) * sin(yaw)
        - sin(pitch)
    ];
    rotMat(:, 2) = [
        cos(yaw) * sin(pitch) * sin(roll) - cos(roll) * sin(yaw)
        cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll)
        cos(pitch) * sin(roll)
    ];
    rotMat(:, 3) = [
        sin(yaw) * sin(roll) + cos(yaw) * cos(roll) * sin(pitch)
        cos(roll) * sin(yaw) * sin(pitch) - cos(yaw) * sin(roll)
        cos(pitch) * cos(roll)
    ];
end
function subplotFunc(x,y1,y1Str,y2,y2Str,y3,y3Str,titleStr)
    figure
    subplot(3,1,1)
            plot(x, y1(1,:), 'r')       %plot z acc
            hold on
            plot(x, y1(2,:), 'g')
            plot(x, y1(3,:), 'b')
            title(y1Str)%'Acceleration')
                subplot(3,1,2)
                plot(x,y2(1,:), 'r')    %plot z vel
                hold on
                plot(x, y2(2,:),'g')
                plot(x, y2(3,:), 'b')
                title(y2Str)%'Velocity')
                    subplot(3,1,3)
                    plot(x, y3(1,:), 'r')%plot z pos
                    hold on
                    plot(x, y3(2,:), 'g')
                    plot(x, y3(3,:), 'b')
                    title(y3Str)%'Position')

                    ha = axes('Position',[0 0.9 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');

    text(0.5, 0.1,titleStr,'HorizontalAlignment' ,'center','VerticalAlignment', 'top');
    movegui('northwest') 

end
% function [inputs integral] = pidController(prevErr, err, integral,timeStep,Kmain,Ti,Td,I)
%     integral = integral + err.*timeStep;
%     integral = 0; %to make it a PD-controller
%     derivative = (err - prevErr )./timeStep;
%     u = Kmain * ( err + (1/Ti)*integral + Td*derivative ) 
%     inputs = convertStateCorrectionsToInputs(u); %?!?!
% end
% function inputs = convertStateCorrectionsToInputs(u)
% end

% function thrust = calcThrust(inputsSquared,kProp) % returns the thrust relative to the body frame
%     thrust = [0 ; 0 ; kProp*sum(inputsSquared)];
% end
% function torques = calcTorques(inputs,L,kProp,b,IMR)
% % TODO
% % Todo Fix second term
% 
%     term1 = b*(input(1)^2 - input(2)^2 + input(3)^2 - input(4)^2);
%     term2 = IMR*(inputsStored(1,counter))
%     torques = [
%         L*kProp(-input(2)^2 + input(4)^2);
%         L*kProp(-input(1)^2 + input(3)^2);
%         term1 + term2
%     ];
% end



