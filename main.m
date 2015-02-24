function main()
  % Constants
    g = 9.82;    % gravity
    m = 1.0;     % mass of copter
    L = 0.25;    % distance to center
    k = 3e-4;    % propeller constant
    b = 0;       % drag coefficent

  % PID-coefficients
    kp = 5.0;
    ki = 2.0;
    kd = 0.06;

  % Initial values
    a = [ 0 ; 0 ; -g ];
    I = [0.025 0 0; 0 0.025 0; 0 0 0.05];
    v0 = zeros(3,1);
    v = v0;
    v2 = v0;
    a2 = a;
    pos = zeros(3,1);

    pos2 = zeros(3,1);

    thetaVec = zeros(3,1);
    
    refHeight = 10;
    errHeight = ones(4,1)*(pos(3)-refHeight); %
    integral = zeros(4,1);
    %inputs = zeros(4,1);
    errHeight2 = ones(4,1)*(pos2(3)-refHeight); %
    integral2 = zeros(4,1);
    
  % Time variables
    h = 0.001; % step length / delta time
    tStart = 0;
    tStop = 100;
    ta = tStart:h:tStop;
    counter = 0;
   
  % Variables to store values for plots  
    posVec = zeros(3,numel(ta));
    velVec = zeros(3,numel(ta));
    accVec = zeros(3,numel(ta));
    
    v2vec  = zeros(3,numel(ta));
    acc2Vec  = zeros(3,numel(ta));
    pos2Vec  = zeros(3,numel(ta));
    
%     angVec = zeros(3,numel(ta));
%     angVec = zeros(3,numel(ta));
%     angVec = zeros(3,numel(ta));
    
  %Variables for another aproach
    aN = a; 
    vN = v;
    pN = pos;
    
    pNVec = posVec;
    vNVec = velVec;
    aNVec = accVec;
    sumAN = zeros(3,1);
    
    eN =  ones(4,1)*(pN(3)-refHeight);
    integralN = 0;
    %inputsN= zeros(4,1);
    %thrustTotN = zeros(4,1);
    asum = 0;
       
    for t = ta;
        counter = counter +1;
       
        errHeightPrev = errHeight;
        errHeight = ones(4,1) * (refHeight - pos(3)); %DETTA M?STE ?NDRAS N?R VI INF?R VINKLAR
        
        errHeightPrev2 = errHeight2;
        errHeight2 = ones(4,1) * (refHeight - pos2(3)); %DETTA M?STE ?NDRAS N?R VI INF?R VINKLAR
        
        [inputs, integral] = pidHeight( kp,ki,kd,errHeight, errHeightPrev, h, integral);
        thrustTot = thrust(k,inputs);
        
        [inputs2, integral2] = pidHeight( kp,ki,kd,errHeight2, errHeightPrev2, h, integral2);
        thrustTot2 = thrust(k,inputs2);
     
        rotMat = rotation( thetaVec );
        
        %TODO uppdatera vinklar
        
        a = acceleration(g, rotMat, thrustTot, m);
        asum = asum + a;
        %v = h*asum;
        v = velocity(a, t, v0);

        
        a2 = acceleration(g, rotMat, thrustTot2, m);
        v2 = v2 + h*a2;
        
        aa = angAcceleration(I, inputs, L, b, k);

        pos = pos + h * v; %Euler
        
        pos2 = pos2 + h * v2; %Euler
        
        v2vec(:,counter) = v2;
        pos2Vec(:,counter) = pos2;
        acc2Vec(:,counter) = a2;
        
        posVec(:,counter) = pos;
        velVec(:,counter) = v;
        accVec(:,counter) = a;
        
%The other aproach
        eNprev = eN;
        eN = refHeight - pN(3);

        [inputsN, integralN] = pidHeightN( eN, eNprev, h, integralN);
        thrustTotN = thrust(k,inputsN);

        aN = -[0;0;g] + (rotMat * [0;0;thrustTotN]) / m;
        sumAN = aN + sumAN;
        vN = h * sumAN;
        pN = pN + vN*h;
        
        pNVec(:,counter) = pN;
        vNVec(:,counter) = vN;
        aNVec(:,counter) = aN;
    end

    subplotFunc(ta, accVec(3,:),velVec(3,:), posVec(3,:), sprintf('\b kp = %f,  ki = %f,  kd = %f',kp,ki,kd)); 
    subplotFunc(ta, acc2Vec(3,:),v2vec(3,:), pos2Vec(3,:), sprintf('\b kp = %f,  ki = %f,  kd = %f',kp,ki,kd)); 
    subplotFunc(ta, aNVec(3,:), vNVec(3,:), pNVec(3,:), sprintf('\b kp = %f,  ki = %f,  kd = %f',kp,ki,kd)); 

    %     figure
%     plot(ta,v2vec(3,:));


end

function acc = acceleration(g , rotMat, thrustTot, m)
    gravity = [0; 0; -g];
    acc = gravity + (rotMat * [0;0;thrustTot]) ./ m; %3x1-vector 
end

% Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.

function tau = torques(inputs, L, b, k)

tau = [
    L*k*(inputs(1)-inputs(3)) 
    L*k*(inputs(2)-inputs(4))
    b*(inputs(1)-inputs(2)+inputs(3)-inputs(4))
    ];
    
end

function angAcc = angAcceleration(I, inputs, L, b, k)
    tau = torques(inputs, L, b, k);                       % vec3 of torques in x,y,z
    angAcc = inv(I)*tau;

end

function angVel = angVelocity(aa, t)
    angVel = aa*t;
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
    derivative = ((eN-eNprev)./h);
    inputN = (kp*eN + ki*integralN + kd*derivative)./4;
    
    control = inputN < 0; % We cant have negative input, therefore
    inputN(control) = 0;  % we set all negative values to zero
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
            plot(x, y1(1,:), 'r')       %plot z acc
            hold on
            plot(x, y1(2,:), 'g')
            plot(x, y1(3,:), 'b')
            title('Acceleration')
                subplot(3,1,2)
                plot(x,y2(1,:), 'r')    %plot z vel
                hold on
                plot(x, y3(2,:),'g')
                plot(x, y3(3,:), 'b')
                title('Velocity')
                    subplot(3,1,3)
                    plot(x, y3(1,:), 'r')%plot z pos
                    hold on
                    plot(x, y3(2,:), 'g')
                    plot(x, y3(3,:), 'b')
                    title('Position')

                    ha = axes('Position',[0 0.9 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');

    text(0.5, 0.1,str,'HorizontalAlignment' ,'center','VerticalAlignment', 'top')

end

