function Final2()
    clear all
    close all
    clc
    % konstanter
    g = 9.82;       % gravitationskonstant
    m = 1;          % massa
    L = 0.25;       % l?ngd fr?n kroppens centrum till mitten av en propeller;
    k = 3e-6;   % propellerkonstant, inkl. lufttryck etc.
    b = 1e-7;          % drag constant
    I = diag([1 1 2].*0.025); % tr?ghetsmoment f?r copterns vridning
%     IMR = 1;                  % rotorernas tr?ghetsmoment
    

    % initialvillkor
    eta      = [ -pi/9; pi/4 ; 0 ]; % vinklar; roll, pitch, yaw
    
    phiDot   = 0; % vinkelhastighet
    thetaDot = 0;
    psiDot   = 0;
    
    etaDot = [phiDot;thetaDot;psiDot];
    
    input = zeros(4,1); % vinkelhastigheter p? varje rotor
    
%     thrust = [0;0;0];         % agerar bara i kroppens lokala och positiva z-led
    
    disturbanceForces = [0.0;0.0;0.0];
    
    pos = [0;0;0]; % globala koord. systemet
    vel = [0;0;0];
    acc = acceleration(g,m,eta,disturbanceForces,0);

    % referenser
    refPos = [  0.5;  -0.5; 0.75];   % Dessa ska kunna ?ndras av anv?ndaren
    psiRef = 0;             % Denna ska kunna ?ndras av anv?ndaren
    psiDotRef = 0;
        
    % Variables for controller
    KPosP = [ 1.0 ; 4.0 ; 4.0 ];
    KPosD = [ 1 ; 5.5 ; 5.50 ];
    KPosDD =[ 0.750 ; 0.75; 0.75 ];
    KAngP = 12.5;
    KAngPY = 12.5;
    KAngD = 1.50;
    KAngDY = 5.0;
    
    % tidsspann, sekunder
    tStart = 0;
    tStop = 30;
    tStep = 0.001;
    time = tStart:tStep:tStop;
    totalSteps = numel(time);

    % vektorer f?r att lagra v?rden f?r plottar
    posStored = [ pos zeros(3, totalSteps-1)];
    velStored = [ vel zeros(3, totalSteps-1)];
    accStored = [ acc zeros(3, totalSteps-1)];
    etaStored = [ radtodeg(eta) zeros(3, totalSteps-1)];

    inputStored = input;
   
    counter = 1;
    c=0;
    tic
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
        
        %Conv. mat. from angVel in global coord. syst. to body coord. syst
        W = [ 1, 0, -sin(theta) ;
              0, cos(phi), cos(theta)*sin(phi);
              0, -sin(phi), cos(theta)*cos(phi)];
          
       if ~exist('phiC') || ~exist('thetaC')
           phiC = asin((dx*sin(psi) - dy*cos(psi))...
            /(dx^2 + dy^2 + (dz+g)^2));
        thetaC = atan((dx*cos(psi) + dy*sin(psi))...
            /(dz+g));
       end
       
        phiCPrev   = phiC;
        thetaCPrev = thetaC;
        
        % Commanded phi & theta.. 
        phiC = asin((dx*sin(psi) - dy*cos(psi))...
            /(dx^2 + dy^2 + (dz+g)^2));
        thetaC = atan((dx*cos(psi) + dy*sin(psi))...
            /(dz+g));
        
        % derivative
        phiDotC    = ( phiCPrev - phiC ) /tStep;
        thetaDotC  = (thetaCPrev-thetaC) /tStep;
        
        phiDot   = etaDot(1);
        thetaDot = etaDot(2);
        psiDot   = etaDot(3);
        
        totalThrust = m*(dx*( sin(theta)*cos(psi)*cos(phi) + sin(psi)*sin(phi) )...
                        +dy*( sin(theta)*sin(psi)*cos(phi) - cos(psi)*sin(phi) )...
                        +(dz+g)*cos(theta)*cos(phi));
                    
        if totalThrust < 0
            c = c+1;
             totalThrust = 0; % the generated thrust cant be negative
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
%         temp = input > 15/k/4; // To limit the possible acceleration 
%         input(temp) = 15/k/4;
        
        % Calculate the angular velocity and acceleration for the body frame
        bAngVel = W*etaDot;
        bAngAcc = I\(tau - cross(bAngVel, I*bAngVel));       
        
        % Uppdatera positioner och vinklar i systemet
        bAngVel   = bAngVel + bAngAcc*tStep;
        etaDot    = W\bAngVel;
        eta       = eta + etaDot*tStep;
        
%         etaDotDot = [0, etaDot(1)*cos(phi)*tan(theta)+etaDot(2)*sin(phi)/(cos(theta)^2), -etaDot(1)*sin(phi)*cos(theta)+etaDot(2)*cos(phi)/(cos(theta)^2);...
%              0, -etaDot(1)*sin(phi), -etaDot(1)*cos(phi);
%              0, (etaDot(1)/cos(theta))*(cos(phi)+sin(phi)*tan(theta)),(-etaDot(1)*sin(phi)+etaDot(2)*cos(phi)*tan(theta))/cos(theta)]...
%              *bAngVel + W\bAngAcc; 
                    
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
    c
    toc
    disp('Done calculating!')
    tic
    subplotFunc(time,accStored,'Acceleration',velStored,'Velocity',posStored,'Position','');
    figure
    plot(time,etaStored(1,:),'r');
    hold on
    plot(time,etaStored(2,:),'g');
    plot(time,etaStored(3,:),'b');
    movegui('northeast') 
    title('Angles')
    legend('roll','pitch','yaw')
    %titleString = sprintf('Kp = %0.2f,  Kd = %0.2f',KAngPY,KAngDY);
    %ha = axes('Position',[0 0.9 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
    %text(0.5, 0.1,titleString,'HorizontalAlignment' ,'center','VerticalAlignment', 'top');
    
    figure
     plot(time,inputStored(1,:),'r');
     hold on
     plot(time,inputStored(2,:),'g');
     plot(time,inputStored(3,:),'b');
     plot(time,inputStored(3,:),'m');
     movegui('south') 
     title('Inputs, angular velocities of each rotor')
     legend('Front rotor','Right rotor','Back rotor','Left rotor')
     
    toc
    disp('Done plotting!')
    animate(posStored, etaStored,refPos,tStep)
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

    legend('x (forwards)','y (sidewards)','z (height)')
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


function animate(posVec, thetaVec,refPos,h)
  % 3D representation
    width = 0.5;
    length = 0.5;
    height = 0.05;

    vertices =  [0 0 0; 0 length 0; width length 0; width 0 0; 0 0 height; 0 length height; width length height; width 0 height];
    vertices(:,1) =  vertices(:,1) - width/2;
    vertices(:,2) =  vertices(:,2) - width/2;
    vertices(:,3) =  vertices(:,3) - height/2;
    vertices = (rotation([0;0;degtorad(45)])*vertices')';
    
    faces = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];

    figure('units','normalized',...
		'position',[0.1, 0.1, 0.7, 0.7]);
    axes('units','normalized',...
    'position',[0.15, 0.15, 0.75, 0.8])
	
	t1 = uicontrol('style','text',...
		'units','normalized',...
		'position',[.45 .0 .2 .05],...
        'horizontalalignment','left',...
		'fontsize',20,...
		'string','Time = 0');
    hold on
    plot(0:1,[0 0],'k->')
    plot([0 0],0:1,'k-^')
    plot3([0 0],[0 0],0:1,'k-^')
    text(1,0,0,'  x')
    text(0,1,0,'  y')
    text(0,0,1,'  z')
    cdata = [0 0 0
             1 1 1
             0 0 0
             1 1 1
             1 0 0
             0 1 0
             0 0 1
             1 1 0];
    maxX = max(posVec(1,:))+0.75;
    maxY = max(posVec(2,:))+0.75;
    maxZ = max(posVec(3,:))+0.75;
    minX = min(posVec(1,:))-0.75;
    minY = min(posVec(2,:))-0.75;
    minZ = min(posVec(3,:))-0.75;
    refVertices = (rotation(degtorad([0 0 0]))*vertices')';
    ref = patch('Vertices', refVertices, 'Faces', faces,'EdgeColor','k','FaceColor','none','LineStyle','--','LineWidth', 0.1);
    set(ref,'xdata', refVertices(:,1) + refPos(1), 'ydata', refVertices(:,2) + refPos(2), 'zdata', refVertices(:,3) + refPos(3),'Faces',faces);
        
    c3d = patch('Vertices', vertices, 'Faces', faces, 'FaceColor','interp','FaceVertexCData',cdata);
    
    rotate3d on
    axis([minX maxX minY maxY minZ maxZ])
    axis equal vis3d
   
    for i = 1:12:numel(posVec(1,:))
        tic
        set(t1,'string',['Time = ', num2str((i-1)*h)]);
        newVertices = (rotation(degtorad(thetaVec(:,i)))*vertices')';
        set(c3d,'xdata', newVertices(:,1) + posVec(1,i), 'ydata', newVertices(:,2) + posVec(2,i), 'zdata', newVertices(:,3) + posVec(3,i),'Faces',faces);
        %rotate(c3d,[0,0,1],1,posVec(:,i));
        pause(0.001)
        toc
    end
end 
