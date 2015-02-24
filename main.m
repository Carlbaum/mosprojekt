function main()
  % Constants
    g = 9.82;
    m = 1;   
    L = 0.25; % Length of each rotor arm.. 
    k = 3e-4; % propeller constant
    
  % PID-coefficients
    kp = 5.0;%3;
    ki = 2.0;%5.5;
    kd = 0.06;%4;

  % Initial values
    a = [ 0 ; 0 ; -g ];
    v0 = zeros(3,1);
    v = v0;
    pos = zeros(3,1);
    thetaVec = zeros(3,1);
    
    refHeight = 10;
    errHeight = ones(4,1)*(pos(3)-refHeight); %
    integral = zeros(4,1);
    %inputs = zeros(4,1);
    
  % Time variables
    h = 0.01; % step length / delta time
    tStart = 0;
    tStop = 1000;
    ta = tStart:h:tStop;
    counter = 0;
   
  % Variables to store values for plots  
    posVec = zeros(3,numel(ta));
    velVec = zeros(3,numel(ta));
    accVec = zeros(3,numel(ta));
    
%   %Variables for another aproach
%     aN = a; 
%     vN = v;
%     pN = pos;
%     
%     pNVec = posVec;
%     vNVec = velVec;
%     aNVec = accVec;
%     sumAN = zeros(3,1);
%     
%     eN = pN(3)-refHeight;
%     integralN = 0;
%     %inputsN= zeros(4,1);
%     %thrustTotN = zeros(4,1);
       
    for t = ta;
        counter = counter +1;
       
        errHeightPrev = errHeight;
        errHeight = ones(4,1) * (refHeight - pos(3)); %DETTA M?STE ?NDRAS N?R VI INF?R VINKLAR
        
        [inputs, integral] = pidHeight( kp,ki,kd,errHeight, errHeightPrev, h, integral);
        thrustTot = thrust(k,inputs);
     
        rotMat = rotation( thetaVec );
        
        %TODO uppdatera vinklar
        
        a = acceleration(g, rotMat, thrustTot, m);
        v = velocity(a, t, v0);
        pos = pos + h * v; %Euler
        
        posVec(:,counter) = pos;
        velVec(:,counter) = v;
        accVec(:,counter) = a;
        
% %The other aproach
%         eNprev = eN;
%         eN = refHeight - pN(3);
% 
%         [inputsN, integralN] = pidHeightN( eN, eNprev, h, integralN);
%         thrustTotN = thrust(k,inputsN);
% 
%         aN = -[0;0;g] + (rotMat * [0;0;thrustTotN]) ./ m;
%         sumAN = aN + sumAN;
%         vN = h * sumAN;
%         pN = pN + vN*h;
%         
%         pNVec(:,counter) = pN;
%         vNVec(:,counter) = vN;
%         aNVec(:,counter) = aN;
    end
    subplotFunc(ta, accVec(3,:),velVec(3,:),posVec(3,:), sprintf('\b kp = %f,  ki = %f,  kd = %f',kp,ki,kd));
%     subplotFunc(ta,  aNVec(3,:), vNVec(3,:), pNVec(3,:), sprintf('\b kp = %f,  ki = %f,  kd = %f',kp,ki,kd));
end

function acc = acceleration(g , rotMat, thrustTot, m)
    gravity = [0; 0; -g];
    acc = gravity + (rotMat * [0;0;thrustTot]) ./ m; %3x1-vector 
end

function vel = velocity(a,t, v0)
    vel = a*t + v0; 
end

function thrustTot = thrust(k, inputs)
    thrustTot = k*sum(inputs.^2); %d?r input ?r en 4x1-vector inneh?llandes de fyra rotorernas vinkelhastighet.
end

function [input,integral] = pidHeight(kp,ki,kd,errHeight, errHeightPrev,h,integral)
    integral = integral + errHeight*h;
    derivative = ((errHeight-errHeightPrev)./h);
    input = (kp*errHeight + ki*integral + kd*derivative)./4;
    
    control = input < 0; % We cant have negative input, therefore
    input(control) = 0;  % we set all negative values to zero
end

function [inputN,integralN] = pidHeightN(eN, eNprev,h,integralN)
    kp = 5.0;%3;
    ki = 2.0;%5.5;
    kd = 0.06;%4;
    integralN = integralN + eN*h;
    derivative = ((eN-eNprev)/h);
    inputN = kp*eN + ki*integralN + kd*derivative;
end

function rotMat = rotation( thetaVec )
    roll    = thetaVec(1);
    pitch   = thetaVec(2);
    yaw     = thetaVec(3);

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

function subplotFunc(x,y1,y2,y3,str)
    figure
    subplot(3,1,1)
            plot(x, y1)       %plot z acc
            title('Acceleration')
                subplot(3,1,2)
                plot(x,y2)    %plot z vel
                title('Velocity')
                    subplot(3,1,3)
                    plot(x, y3)%plot z pos
                    title('Position')
                    ha = axes('Position',[0 0.9 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');

    text(0.5, 0.1,str,'HorizontalAlignment' ,'center','VerticalAlignment', 'top')
end

