function new()
%close all
  % Constants
    g = 9.82;    % gravity
    m = 1.0;     % mass of copter
    L = 0.25;    % distance to center
    k = 3e-6;    % propeller constant
    b = 1e-7;    % drag coefficent

  % PID-coefficients (Height)
    kp = 0.75;
    ki = 0.570;
    kd = 0.03;
  % PID-coefficients (angles)
    Kp = 0.0250;
    Ki = 0.020;
    Kd = 0.0006;

  % Initial values
    acc = [ 0 ; 0 ; -g ];
    I = [0.025 0 0; 0 0.025 0; 0 0 0.05];
    v0 = zeros(3,1);
    vel = v0;
    pos = [0; 0; 0];
    theta = [degtorad(25);degtorad(30);degtorad(-29)]; % angles; [roll;pitch;yaw]
    angVel = zeros(3,1);
    angAcc = zeros(3,1);    
    
    refHeight = 10;
    %errHeight = refHeight-pos(3); %ones(4,1)*(pos(3)-refHeight); %
    refPos = [0;0;12];
    errGlobalPos = refPos - pos;
    errLocalPos = 0;
    
    refAngles = zeros(3,1);
    errAngles = refAngles - theta;
    
    %pidIntegral = zeros(3,1);
    %heightIntegral = 0;
    posIntegral = zeros(3,1);
    localPosIntegral = zeros(3,1);
    globalPosIntegral = zeros(3,1);
    anglesIntegral = zeros(3,1);
    
  % Time variables
    h = 0.01; % step length / delta time
    tStart = 0;
    tStop = 25.0;
    ta = tStart:h:tStop;
    counter = 0;
   
  % Preallocate variables to store values for plots  
    posVec = zeros(3,numel(ta));
    velVec = zeros(3,numel(ta));
    accVec = zeros(3,numel(ta));
    thetaVec = zeros(3,numel(ta));
    angAccVec = zeros(3,numel(ta));
    angVelVec = zeros(3,numel(ta));
       
    for t = ta;
        counter = counter +1;
        
        thetaVec(:,counter) = theta;
        angVelVec(:,counter) = angVel;
        angAccVec(:,counter) = angAcc;
        
        posVec(:,counter) = pos;
        velVec(:,counter) = vel;
        accVec(:,counter) = acc;

       % errHeightPrev = errHeight;
       % errHeight = refHeight - pos(3);
        rotMat = rotation( theta );
        errGlobalPosPrev = errGlobalPos; 
        errGlobalPos = refPos - pos; % is a vector from the body center to the reference point
        errLocalPosPrev = rotMat\errGlobalPosPrev; 
        errLocalPos = rotMat\errGlobalPos; % should be the same vector, but expressed in the local coordinate system
        
%         errAnglesPrev = errAngles;
%         errAngles = refAngles - theta;

        %[thrustTot, heightIntegral] = pidHeight( kp,ki,kd,errHeight, errHeightPrev, h, heightIntegral);
        [thrustTotLocal, localPosIntegral] = pidPos( kp,ki,kd,errLocalPos, errLocalPosPrev, h, localPosIntegral);
        [thrustTotGlobal, globalPosIntegral] = pidPos( kp,ki,kd,errGlobalPos, errGlobalPosPrev, h, globalPosIntegral);
        %[tau, anglesIntegral] = pidAngles(Kp, Ki,Kd, errAngles, errAnglesPrev, h, anglesIntegral);
        %thrustTotGlobal = rotMat*thrustTotLocal;
        %thrustTotLocal =  rotMat\thrustTotGlobal; % ty R*local = global
        
        inputs(1) = thrustTotGlobal(3)/4 - thrustTotLocal(2)/2; % - yled om input(1) ?kar, d?rf?r 
        inputs(3) = thrustTotGlobal(3)/4 + thrustTotLocal(2)/2; % + yled
        inputs(2) = thrustTotGlobal(3)/4 - thrustTotLocal(1)/2; % + xled
        inputs(4) = thrustTotGlobal(3)/4 + thrustTotLocal(1)/2; % - xled
        
%         inputs(1) = thrustTot(3)/4 + tau(1)/2;% - thrustTot(2)/2; % -yled om input(1) ?kar, d?rf?r 
%         inputs(3) = thrustTot(3)/4 - tau(1)/2;% + thrustTot(2)/2;
%         inputs(2) = thrustTot(3)/4 + tau(2)/2;% - thrustTot(1)/2;
%         inputs(4) = thrustTot(3)/4 - tau(2)/2;% + thrustTot(1)/2;
        
%         if (max(abs(angVel)) > 0.01)
%             windUpConst = 1;
%         else
%             windUpConst = 1;
%         end
%         
          % thrustTot = (m * g) / (cos(theta(1))*cos(theta(2))*k);
        %pidIntegral = pidIntegral .* windUpConst + theta .* h;
       % e = Kp * theta + Ki * pidIntegral + Kd * angVel ;
        
%         inputs(1) = thrustTot/4 - (e(3) * I(9))/(4 * b) - (e(1) * I(1))/(2 * k * L);
%         inputs(3) = thrustTot/4 - (e(3) * I(9))/(4 * b) + (e(1) * I(1))/(2 * k * L);
%         inputs(2) = thrustTot/4 + (e(3) * I(9))/(4 * b) - (e(2) * I(5))/(2 * k * L);
%         inputs(4) = thrustTot/4 + (e(3) * I(9))/(4 * b) + (e(2) * I(5))/(2 * k * L);
%         
        
        %[t, integral] = pidHeight( kp,ki,kd,errHeight, errHeightPrev, h, integral);
        %thrustTot = thrust(k,inputs);
        
        %Vinkelacceleration till vinklar
%         angAcc = angAcceleration(I, L, b, k,inputs);
        angAcc = angAcceleration(I, L, b, k,inputs);%tau);
        angVel = angVelocity(angAcc, t);
        theta = theta + h*angVel;   %Euler
        
        %Acceleration till position
        %acc = acceleration(g, rotMat, thrustTotLocal, m);
        acc = accelerationNew(g, thrustTotGlobal, m);
        vel = velocity(acc, t, v0);
        pos = pos + h * vel; %Euler
    end

    %Plot acceleration, velocity and position
    subplotFunc(ta, accVec,velVec, posVec, sprintf('\b kp = %f,  ki = %f,  kd = %f',kp,ki,kd),1); 
    %Plottar vinklar, vinkelhastighet, vinkelacceleration
    subplotFunc(ta, angAccVec,angVelVec, radtodeg(thetaVec), sprintf('\b kp = %f,  ki = %f,  kd = %f',Kp,Ki,Kd),2); 


end

function acc = acceleration(g , rotMat, thrustTot, m)
    gravity = [0; 0; -g];
    acc = gravity + (rotMat * thrustTot) ./ m; %3x1-vector 
end

function acc = accelerationNew(g , thrustTot, m)
    gravity = [0; 0; -g];
    acc = gravity + thrustTot ./ m; %3x1-vector 
end

% Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.

function tau = torques(inputs, L, b, k)

    tau = [
        L*k*(inputs(1)-inputs(3)) 
        L*k*(inputs(2)-inputs(4))
        b*(inputs(1)-inputs(2)+inputs(3)-inputs(4))
    ];
    
end

function angAcc = angAcceleration(I, L, b, k,inputs)%tau)%inputs)
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
    input = (kp*errHeight + ki*integral + kd*derivative);%./4;
    
    control = input < 0; % We cant have negative input, therefore
    input(control) = 0;  % we set all negative values to zero
end

function [input,posIntegral] = pidPos(kp,ki,kd,errPos, errPosPrev,h,posIntegral)
    % Compute error and inputs.
    if max(abs(posIntegral(:)) > 0.01)
        posIntegral(:) = 0;
    end
    
    posIntegral = posIntegral + errPos*h;
    derivative = ((errPos-errPosPrev)./h);
    input = (kp*errPos + ki*posIntegral + kd*derivative);%./4;
    
%     control = input < 0; % We cant have negative input, therefore
%     input(control) = 0;  % we set all negative values to zero
end

function [input,integral] = pidAngles(kp,ki,kd,errAngles, errAnglesPrev,h,integral)
    integral = integral + errAngles*h;
    derivative = ((errAngles-errAnglesPrev)./h);
    input = (kp*errAngles + ki*integral + kd*derivative);
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

function subplotFunc(x,y1,y2,y3,str,val)
    figure
    subplot(3,1,1)
            plot(x, y1(1,:), 'r')       %plot z acc
            hold on
            plot(x, y1(2,:), 'g')
            plot(x, y1(3,:), 'b')
            if (val == 1)
               title('Acceleration')
            end
                    
            if(val == 2)
                title('angular acceleration (rad/s^2)')
            end 
            
                subplot(3,1,2)
                plot(x,y2(1,:), 'r')    %plot z vel
                hold on
                plot(x, y2(2,:),'g')
                plot(x, y2(3,:), 'b')
                if (val == 1)
                        title('Velocity')
                        legend('x (forwards)','y (sideways)','z (height)', 'Location', 'NorthEastOutside')
                    end
                    
                    if(val == 2)
                         title('angular velocity (rad/s)')
                    end  
            
                    subplot(3,1,3)
                    plot(x, y3(1,:), 'r')%plot z pos
                    hold on
                    plot(x, y3(2,:), 'g')
                    plot(x, y3(3,:), 'b')   
                    if (val == 1)
                        title('Position')
                    end
                    
                    if(val == 2)
                         title('angle (degrees)')
                    end  

                        
                    ha = axes('Position',[0 0.9 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');

    text(0.5, 0.1,str,'HorizontalAlignment' ,'center','VerticalAlignment', 'top')
    if val ~= 2 
        movegui('northwest');
    else
        movegui('northeast');
    end

end

