function main() %initial values

    g = 9.82;

    a = [ 0 ; 0 ; -g ];
    v0 = zeros(3,1);
    v = v0;
    pos = zeros(3,1);
    thetaVec = zeros(3,1);
    
    refHeight = 10;
    
    an = a; 
    vn = v;
    pn = pos;

    m = 1;   
    L = 0.25;
    k = 3e-6;

    Ftemp = [0 ;0 ;10.8];
    Ftempn = [0 ;0 ;10.8];
    Fnew = zeros(3,1);
    
    h = 0.01;
    ta = 0:h:2000;
    counter = 0;
    
    posVec = zeros(3,numel(ta));
    velVec = zeros(3,numel(ta));
    accVec = zeros(3,numel(ta));
    
    pnVec = posVec;
    vnVec = velVec;
    anVec = accVec;
    sumA = zeros(3,1);
    
    e = pos(3)-refHeight;
    integral = 0;
    inputs= zeros(4,1);
    thrustTot = zeros(4,1);
    rotMat = zeros(3);
    
    for t = ta;
        counter = counter +1;
       
        eprev = e;
        e = pos(3) - refHeight;
        
        [inputs, integral] = pidHeight( e, eprev, h, integral);
        
        thrustTot = thrust(k,inputs);
        
        rotMat = rotation( thetaVec );
        
        %uppdatera vinklar
        %accelerationen 
        
        %if vn(3) ~= 0
           % Ftempn(3) = Ftempn(3) - 0.001*vn(3);
        %end
        
        %if v(3) ~= 0
            %Ftemp(3) = Ftemp(3) - 0.001*v(3);
        %end
        
        a = acceleration(g, rotMat, thrustTot, m);
        v = velocity(a, t, v0);
        
        %an = Ftempn/m -[0;0;g];
        an = -[0;0;g] + (rotMat * [0;0;thrustTot]) ./ m;
        sumA = an + sumA;
        vn = h * sumA;
        pn = pn + vn*h;
        
        pos = pos + h * v; %Euler
        
        posVec(:,counter) = pos;
        velVec(:,counter) = v;
        accVec(:,counter) = a;
        
        pnVec(:,counter) = pn;
        vnVec(:,counter) = vn;
        anVec(:,counter) = an;
    end
    
    figure
        subplot(1,3,1)
        plot(ta, accVec(3,:))       %plot z acc
        title('Acceleration')
            subplot(1,3,2)
            plot(ta,velVec(3,:))    %plot z vel
            title('Velocity')
                subplot(1,3,3)
                plot(ta, posVec(3,:))%plot z pos
                title('Position')
                
    figure
        subplot(1,3,1)
        plot(ta, anVec(3,:))         %plot z acc
        title('Acceleration')
            subplot(1,3,2)
            plot(ta,vnVec(3,:))      %plot z vel
            title('Velocity')
                subplot(1,3,3)
                plot(ta, pnVec(3,:)) %plot z pos
                title('Position')
end

function acc = acceleration(g , rotMat, thrustTot, m)
    gravity = [0; 0; -g];
    %acc = gravity + Ftemp / m;
    acc = gravity + (rotMat * [0;0;thrustTot]) ./ m; %3x1-vector 
end

function vel = velocity(a,t, v0)
    vel = a*t + v0; 
end

function thrustTot = thrust(k, inputs)
    thrustTot = k*sum(inputs.^2); %d?r input ?r en 4x1-vector inneh?llandes de fyra rotorernas vinkelhastighet.
end

function [input,integral] = pidHeight(e, eprev,h,integral)
    kp = 0.1;%3;
    ki = 0.1;%5.5;
    kd = 0.1;%4;
    integral = integral + e*h;
    derivative = ((e-eprev)/h);
    input = kp*e + ki*integral + kd*derivative;
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

