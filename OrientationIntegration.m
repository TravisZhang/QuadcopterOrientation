function [T,Y] = OrientationIntegration (id,t_end,ic,options)
    % This is the main function which performs a numerical intigration
    % for the orientation using ode solver (ode45 is encouraged).
    % inputs: id -- string type: either 'space-three 1-2-3' or 'body-two 1-2-1'
    %         t_end -- integer, indicate the finishing time
    %         ic -- initial angles, a row vector of three components
    %         options -- solver options, used when call the solver
    %
    % Author:      Haohan Zhang
    % Affiliation: ROAR Lab @ Columbia
    % Date:        1/23/2016
       
    % (1) call numerical solver and return the time and state variables.
    % call the time vector as 'T' and state variables as 'Y'. 
    % Hint: You might want to add some lines of codes to match up you
    % initial conditions and your selected rotation.
    if strcmp(id,'space-three 1-2-3')        
    %    ic1 = space123ToBody121(ic);   
    % if the initial condition is given in space frame,change it in to body
    % frame
        [T,Y] = ode45(@(t,y) odeOfRotation(t,y,id),[0 t_end],ic,options); % uncomment this line and fill in the ...
    elseif strcmp(id,'body-two 1-2-1')
        [T,Y] = ode45(@(t,y) odeOfRotation(t,y,id),[0 t_end],ic,options); % uncomment this line and fill in the ...        
    end
    % (2) plot the angles according to the representation
    plotAngle(T,Y,id);
    
    % (3) animate the copter
    animateQuadcopter(T,Y,id);
end

function dy = odeOfRotation(t,y,id)
    % Given the angular velocity history data and initial conditions, solve for
    % the current rotation angles of the quadcopter.
    %
    % State vaiables y = [theta1,theta2,theta3] are three Euler angles.
    % Omega =[omega1 omega2 omega3] represent the angular velocity of B in A.
    %
    % input - t: time vector
    %         y: state variables (angles)
    %         id: rotation type: 'body-two 1-2-1' or 'space-three 1-2-3'
    % output - dy: angle rates

    % (1) asssign the values/functions of angular velocity.
    
    % (2) You need to finish the following to switch the case for different
    % id. Set up the odes based on the handout.
    switch id
       case 'body-two 1-2-1'
           % fill in differential equations here
           dy = zeros(3,1);
           dy(1)=(2*sin(2*t)*sin(y(3))+3*sin(3*t)*cos(y(3)))/sin(y(2));
           dy(2)=2*sin(2*t)*cos(y(3))-3*sin(3*t)*sin(y(3));
           dy(3)=sin(t)-(2*sin(2*t)*sin(y(3))+3*sin(3*t)*cos(y(3)))*cos(y(2))/sin(y(2));

       case 'space-three 1-2-3'
           % fill in differential equations here
           dy = zeros(3,1);
           dy(1)=sin(t)+(2*sin(2*t)*sin(y(1))+3*sin(3*t)*cos(y(1)))*sin(y(2))/cos(y(2));
           dy(2)=2*sin(2*t)*cos(y(1))-3*sin(3*t)*sin(y(1));
           dy(3)=(2*sin(2*t)*sin(y(1))+3*sin(3*t)*cos(y(1)))/cos(y(2));
           
       otherwise
           error('Not a legal rotation representation! Please check your input.')
    end
end

% rotation helpers
function R = rotationSpace123(phi)
    % this function returns the rotation matrix based on space three 1-2-3
    % convention.
    % input: three rotation angles
    % output: rotation matrix
    
    phi1 = phi(1); phi2 = phi(2); phi3 = phi(3);
    R = [ cos(phi2)*cos(phi3), sin(phi1)*sin(phi2)*cos(phi3) - sin(phi3)*cos(phi1), cos(phi1)*sin(phi2)*cos(phi3) + sin(phi3)*sin(phi1) ;
          cos(phi2)*sin(phi3), sin(phi1)*sin(phi2)*sin(phi3) + cos(phi3)*cos(phi1), cos(phi1)*sin(phi2)*sin(phi3) - cos(phi3)*sin(phi1) ;
         -sin(phi2)            , sin(phi1)*cos(phi2)                                      , cos(phi1)*cos(phi2)                                      ];
end

function phi = body121ToSpace123(theta)
    % this function converts the angles from space three 1-2-3 to body two
    % 1-2-1.
    R = rotationBody121(theta);
    phi = zeros(3,1);
    phi(2) = -asin(R(3,1));
    phi(3) = atan2(R(2,1) , R(1,1));
    phi(1) = atan2(R(3,2) , R(3,3));
    
end

function R = rotationBody121(theta)
    % this function returns the rotation matrix based on body three 1-2-1
    % convention.
    % input: three rotation angles
    % output: rotation matrix
    theta1 = theta(1); theta2 = theta(2); theta3 = theta(3);
    R = [ cos(theta2)            ,  sin(theta2)*sin(theta3)                                      ,  sin(theta2)*cos(theta3)                                       ;
          sin(theta1)*sin(theta2), -sin(theta1)*cos(theta2)*sin(theta3) + cos(theta3)*cos(theta1), -sin(theta1)*cos(theta2)*cos(theta3) - sin(theta3)*cos(theta1) ;
         -cos(theta1)*sin(theta2),  cos(theta1)*cos(theta2)*sin(theta3) + cos(theta3)*sin(theta1),  cos(theta1)*cos(theta2)*cos(theta3) - sin(theta3)*sin(theta1)];
end

function theta = space123ToBody121(phi)
    % this function converts the angles from body two 1-2-1 to space three
    % 1-2-3.
    R = rotationSpace123(phi);
    theta = zeros(3,1);
    theta(2) = acos(R(1,1));
    theta(3)= atan2(R(1,2) , R(1,3));
    theta(1)= atan2(R(2,1) , -R(3,1));
    
end

% plot routine
function plotAngle (T,Y,id)
    % this function plots the orientations of the quadcopter.
    if strcmp(id,'body-two 1-2-1')
        lgd = {'\theta_1','\theta_2','\theta_3'};
    elseif strcmp(id,'space-three 1-2-3')
        lgd = {'\phi_1','\phi_2','\phi_3'};
    end
    figure
    plot(T,Y(:,1)*180/pi,'-',T,Y(:,2)*180/pi,'-.',T,Y(:,3)*180/pi,'--','linewidth',2)
    title('Orientation of a quadcopter')
    xlabel('Time (s)','fontname','times new roman','fontsize',20)
    ylabel('Magnitude (deg)','fontname','times new roman','fontsize',20)
    legend(lgd,'location','northeast',...
        'orientation','horizontal','Fontsize',20)
    set(gca,'fontsize',20, 'fontname','times new roman')
end

function A = getQuadcopter(Yc,id)
    % this function gets the position of the quadcopter B in the inertial
    % frame. Its shape is simplified to be a cross.
    % input: Yc - the current rotation angles
    % output: A - the position of coners of the quadcopter in the inertial
    %             frame
    
    B = 20*[-1 1  0 0 ;
             0 0 -1 1 ;
             0 0  0 0];  % The four coners of the cross in body frame
   
    % pick rotations
    if strcmp(id,'body-two 1-2-1')
        R = rotationBody121(Yc);
    elseif strcmp(id,'space-three 1-2-3')
        R = rotationSpace123(Yc);
    end
    
    A = zeros(3,4); % initialize A
    for i = 1:4 % transform the corners into the inertial frame
        A(:,i) = R * B(:,i);
    end
end

function plotTrajectory(T,Y,id,n)
    % this function plots the trajectories of one of the four corners of
    % the quadcopter computed by the two different orientation
    % representations. input n represents the nth corner of the copter. The
    % purpose of this funciton is to check if the program runs correctly.
    % Since using different rotation representation should not produce the
    % different orientation of the copter, the position of the corner
    % should remain identical with either representation. Additionally,
    % because there is no translations in this case, the transformation
    % from the inertial frame to the body frame only contains rotation.
    
    A = zeros(length(T),3,4); % initialize position in inertial frame
    for i = 1:length(T)
        A(i,:,:) = getQuadcopter(Y(i,:),id);
    end

    figure
    plot(T,A(:,1,n),T,A(:,2,n),T,A(:,3,n))
    title(strcat('Postion of a corner of the quadcopter'))
    xlabel('Time (s)','fontname','times new roman','fontsize',20)
    ylabel('Magnitude (cm)','fontname','times new roman','fontsize',20)
    legend({'x','y','z'},'location','northeast',...
        'orientation','horizontal','Fontsize',20)
    set(gca,'fontsize',20, 'fontname','times new roman')
end

function animateQuadcopter(T,Y,id)
    % this function animates the orientation of the quadcopter. It uses
    % drawnow, without any video outputs.
    figure
    for i = 1:length(T)
        A = getQuadcopter(Y(i,:),id);
        
        % draw the two lines of the cross of the copter
        x = [A(:,1) A(:,2)].';
        y = [A(:,3) A(:,4)].';
        plot3(x(:,1),x(:,2),x(:,3),'color','b','linewidth',4);
        hold on
        plot3(y(:,1),y(:,2),y(:,3),'color','b','linewidth',4);
        plot3(0,0,0,'k.','markersize',60) % draw a circle to represent center
        axis([-25,25,-25,25,-25,25]) % fixed axes range
        
        drawnow
        pause(0.05) % delay 0.05s
        clf % clear current figure
    end
end