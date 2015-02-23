function main() %initial values

    g = 9.82;

    a = [ 0 ; 0 ; -g ];
    v0 = zeros(3,1);
    v = v0;
    pos = zeros(3,1);
    
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
    ta = 0:h:100;
    counter = 0;
    
    
    posVec = zeros(3,numel(ta));
    velVec = zeros(3,numel(ta));
    accVec = zeros(3,numel(ta));
    
    pnVec = posVec;
    vnVec = velVec;
    anVec = accVec;
    sumA = zeros(3,1);
    
    for t = ta;
        counter = counter +1;
        
        if vn(3) ~= 0
            Ftempn(3) = Ftempn(3) - 0.001*vn(3);
        end
        
        if v(3) ~= 0
            Ftemp(3) = Ftemp(3) - 0.001*v(3);
        end
        
        a = acceleration(g, Ftemp, m);
        v = velocity(a, t, v0);
        
        an = Ftempn/m -[0;0;g];
        sumA = an+ sumA;
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

function acc = acceleration(g , Ftemp, m)
    gravity = [0; 0; -g];
    acc = gravity + Ftemp / m;
    acc = 
end

function vel = velocity(a,t, v0)
    vel = a*t + v0; 
end

