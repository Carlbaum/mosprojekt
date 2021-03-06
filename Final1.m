function Final1()
close all
clear all
clc
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
    theta = [degtorad(-25);degtorad(45);degtorad(-0)]; % angles; [roll;pitch;yaw]
    angVel = zeros(3,1);
    angAcc = zeros(3,1);    
    
    refHeight = 10;
    errHeight = refHeight-pos(3); %ones(4,1)*(pos(3)-refHeight); %
%    refPos = [0;0;refHeight];
     
    refAngles = zeros(3,1);
    errAngles = refAngles - theta;
    
    heightIntegral = 0;
    anglesIntegral = zeros(3,1);
    
  % Time variables
    h = 0.001; % step length / delta time
    tStart = 0;
    tStop = 10.0;
    ta = tStart:h:tStop;
    counter = 0;
    

    
   
  % Preallocate variables to store values for plots  
    posVec = zeros(3,numel(ta));
    velVec = zeros(3,numel(ta));
    accVec = zeros(3,numel(ta));
    thetaVec = zeros(3,numel(ta));
    angAccVec = zeros(3,numel(ta));
    angVelVec = zeros(3,numel(ta));
    tic   
    for t = ta;
        counter = counter +1;
        
        %store values for plots
        thetaVec(:,counter) = theta;
        angVelVec(:,counter) = angVel;
        angAccVec(:,counter) = angAcc;
        
        posVec(:,counter) = pos;
        velVec(:,counter) = vel;
        accVec(:,counter) = acc;

        %calculate errors
        errHeightPrev   = errHeight;
        errHeight       = refHeight - pos(3);
        
        errAnglesPrev   = errAngles;
        errAngles       = refAngles - theta;
        
        rotMat = rotation( theta );
        
        [thrustTot, heightIntegral] = pidHeight( kp,ki,kd,errHeight, errHeightPrev, h, heightIntegral);
        [tau, anglesIntegral] = pidAngles(Kp, Ki, Kd, errAngles, errAnglesPrev, h, anglesIntegral);
        
        inputs(1) = thrustTot(1)/4 + tau(1)/2;% - thrustTot(2)/2;  
        inputs(3) = thrustTot(1)/4 - tau(1)/2;% + thrustTot(2)/2;
        inputs(2) = thrustTot(1)/4 + tau(2)/2;% - thrustTot(1)/2;
        inputs(4) = thrustTot(1)/4 - tau(2)/2;% + thrustTot(1)/2;
        angAcc = angAcceleration(I, L, b, k,tau);%inputs);%tau);
        angVel = angVelocity(angAcc, t);
        theta = theta + h*angVel;   %Euler
        
        %Acceleration till position
        acc = acceleration(g, rotMat, thrustTot, m);
        vel = velocity(acc, t, v0);
        pos = pos + h * vel;
    end
    toc
    disp('Done calulating!')

    %Plot acceleration, velocity and position
    subplotFunc(ta, accVec,velVec, posVec, sprintf('\b kp = %f,  ki = %f,  kd = %f',kp,ki,kd),1); 
    %Plottar vinklar, vinkelhastighet, vinkelacceleration
    subplotFunc(ta, angAccVec,angVelVec, radtodeg(thetaVec), sprintf('\b kp = %f,  ki = %f,  kd = %f',Kp,Ki,Kd),2); 
    animate(posVec, thetaVec,h)
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

function angAcc = angAcceleration(I, L, b, k,tau)%inputs)
    %tau = torques(inputs, L, b, k);                       % vec3 of torques in x,y,z
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
               title('Acceleration (m/s^2)')
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
                        title('Velocity (m/s)')
                        legend('x (forwards)','y (sideways)','z (height)', 'Location', 'NorthEastOutside')
                    end
                    
                    if(val == 2)
                        title('angular velocity (rad/s)')
                        legend('Rotation around x-axis','Rotation around y-axis','Rotation around z-axis', 'Location', 'NorthEastOutside')
                    end  
            
                    subplot(3,1,3)
                    plot(x, y3(1,:), 'r')%plot z pos
                    hold on
                    plot(x, y3(2,:), 'g')
                    plot(x, y3(3,:), 'b')   
                    if (val == 1)
                        title('Position (m)')
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

function animate(posVec, thetaVec,h)
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
    cdata = [0 0 1
             1 0 0
             0 1 0
             1 1 0
             0 0 1
             1 0 0
             0 1 0
             1 1 0];
    maxX = max(posVec(1,:))+0.75;
    maxY = max(posVec(2,:))+0.75;
    maxZ = max(posVec(3,:))+0.75;
    minX = min(posVec(1,:))-0.75;
    minY = min(posVec(2,:))-0.75;
    minZ = min(posVec(3,:))-0.75;
    c3d = patch('Vertices', vertices, 'Faces', faces, 'FaceColor','interp','FaceVertexCData',cdata);
  
    rotate3d on
    axis([minX maxX minY maxY minZ maxZ])
    axis equal vis3d
    for i = 1:numel(posVec(1,:))-3
        set(t1,'string',['Time = ', num2str((i-1)*h)]);
        newVertices = (rotation(thetaVec(:,i))*vertices')';
        set(c3d,'xdata', newVertices(:,1) + posVec(1,i), 'ydata', newVertices(:,2) + posVec(2,i), 'zdata', newVertices(:,3) + posVec(3,i),'Faces',faces);
        %rotate(c3d,[0,0,1],1,posVec(:,i));
        pause(0.0001)
    end
end 

