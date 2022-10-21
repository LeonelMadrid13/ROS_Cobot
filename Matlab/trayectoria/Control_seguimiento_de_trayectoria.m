clc  % Clear Command Window
clear %Remove items from workspace, freeing up system memory
close all % Remove all figure

ts = 0.1; % sample time
t  = 0:ts:80; % time array 
N  = length(t) ; % samples

%%%%%%%%%%%%%%%%%%%%%%%%%% Initial conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
% Initial conditions of unicycle mobile manipulator
     
x1(1) = 1;  % initial position (x axis) in meters (m)
y1(1) = -2; % initial position (y axis) in meters (m)
phi(1)=0;  % initial orientation of mobile platform in radians (rad)     
q1(1)=0;    % initial angular position q1 in radians
q2(1)=pi/4; % initial angular position q2 in radians
q3(1)=0;    % initial angular position q3 in radians

a=0.2;     % distance to interest point (manipulator base) in meters (m)
b=0.25;     % platform height 
c=0.25;     % height of arm base
l1=0.5;     % joint length 1
l2=0.55;    % joint length 2

% Geometric Model of Mobile Manipulator

xp(1)=x1(1)+a*cos(phi(1)); % initial platform position (manipulator base in x axis) in meters (m)
yp(1)=y1(1)+a*sin(phi(1)); % initial platform position (manipulator base in y axis) in meters (m)

hx(1)=xp(1)+l1*cos(q2(1))*cos(phi(1)+q1(1))+l2*cos(q2(1)+q3(1))*cos(phi(1)+q1(1));   % initial interest position (gripper point in x axis) in meters (m)
hy(1)=yp(1)+l1*cos(q2(1))*sin(phi(1)+q1(1))+l2*cos(q2(1)+q3(1))*sin(phi(1)+q1(1));   % initial interest position (gripper point in y axis) in meters (m)
hz(1)=b+c+l1*sin(q2(1))+l2*sin(q2(1)+q3(1)); % initial interest position (gripper point in z axis) in meters (m)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Desired Trajectory %%%%%%%%%%%%%%%%%%%%%%%%

%  Sine Trajectory  
hxd = 0.6+0.1*t;     %  Desired Trajectory  (x axis) in meters
hyd = 2*sin(0.1*t);  %  Desired Trajectory  (y axis) in meters
hzd = 1*ones(1,length(t));   %  Desired Trajectory  (z axis) in meters

hxdp = 0.1*ones(1,length(t));   %  Desired velocity  (x axis) in meters/seconds
hydp = 2*0.1*cos(0.1*t);        %  Desired velocity  (y axis) in meters/seconds
hzdp = 0*ones(1,length(t));   %  Desired velocity  (z axis) in meters/seconds

%  Circle Trajectory  
% hxd = 2*cos(0.1*t);    %  Desired Trajectory  (x axis) in meters
% hyd = 2*sin(0.1*t);    %  Desired Trajectory  (y axis) in meters
% 
% hxdp = -2*0.1*sin(0.1*t);       %  Desired velocity  (x axis) in meters/seconds
% hydp = 2*0.1*cos(0.1*t);        %  Desired velocity  (y axis) in meters/seconds

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hxe = zeros(1,N);    % error control (x axis) in meters (m)
hye = zeros(1,N);    % error control (y axis) in meters (m)
hze = zeros(1,N);    % error control (y axis) in meters (m)

u = zeros(1,N); % linear velocity of robot 1  (x axis)
w  = zeros(1,N); % angular velocity of robot 1
q1p = zeros(1,N); % first angular velocity
q2p = zeros(1,N); % second joint velocity
q3p = zeros(1,N); % third joint velocity


for k=1:N  
    
%%%%%%%%%%%%%%%%%%% Controller of Mobile Manipulator %%%%%%%%%%%%%%%%%%%%%%%%% 

        hxe(k) = hxd(k) - hx(k);    % error positon (x axis) in meters        
        hye(k) = hyd(k) - hy(k);    % error positon (y axis) in meters    
        hze(k) = hzd(k) - hz(k);       % error positon (z axis) in meters
        
        he = [hxe(k) hye(k) hze(k)]';  % error array of mobile manipulator 
        
        % Jacobian Matrix of mobile manipulator 
     
        J11 = cos(phi(k));
        J12 = -a*sin(phi(k))-l1*cos(q2(k))*sin(phi(k)+q1(k))-l2*cos(q2(k)+q3(k))*sin(phi(k)+q1(k));
        J13 = -l1*cos(q2(k))*sin(phi(k)+q1(k))-l2*cos(q2(k)+q3(k))*sin(phi(k)+q1(k));
        J14 = -l1*sin(q2(k))*cos(phi(k)+q1(k))-l2*sin(q2(k)+q3(k))*cos(phi(k)+q1(k));
        J15 = -l2*sin(q2(k)+q3(k))*cos(phi(k)+q1(k));
                
        
        J21 = sin(phi(k));
        J22 = a*cos(phi(k))+l1*cos(q2(k))*cos(phi(k)+q1(k))+l2*cos(q2(k)+q3(k))*cos(phi(k)+q1(k));
        J23 = l1*cos(q2(k))*cos(phi(k)+q1(k))+l2*cos(q2(k)+q3(k))*cos(phi(k)+q1(k));
        J24 = -l1*sin(q2(k))*sin(phi(k)+q1(k))-l2*sin(q2(k)+q3(k))*sin(phi(k)+q1(k));
        J25 = -l2*sin(q2(k)+q3(k))*sin(phi(k)+q1(k));
        
        J31 = 0;
        J32 = 0;
        J33 = 0;
        J34 = l1*cos(q2(k))+l2*cos(q2(k)+q3(k));
        J35 = l2*cos(q2(k)+q3(k));
        
        

        J =[J11 J12 J13 J14 J15;...
            J21 J22 J23 J24 J25;...
            J31 J32 J33 J34 J35];
       
        % Control parameters (Diagonal Matrix)
        K= 3*[1 0 0;...
             0 1 0;...
             0 0 1];
         
        hdp=[hxdp(k) hydp(k) hzdp(k)]';
        
        qp = pinv(J)*(hdp+K*tanh(he));  % Trajectory Control based on Lyapunov
        
        % Separate control actions
        u(k)   = qp(1);
        w(k)   = qp(2);
        q1p(k)  = qp(3);
        q2p(k)  = qp(4);
        q3p(k)  = qp(5);
        
        % Apply control actions to Model Robot or Real Robot

        phi(k+1)=phi(k)+w(k)*ts;

        x1p(k)=u(k)*cos(phi(k+1)); % real velocity in meters/seconds
        y1p(k)=u(k)*sin(phi(k+1)); % real velocity in meters/seconds
        

        % numerical integral (Euler Method)
        x1(k+1)=x1(k)+ts*x1p(k);  % real position (x axis) in meters (m)
        y1(k+1)=y1(k)+ts*y1p(k);  % real position (y axis) in meters (m)
        
        q1(k+1)=q1(k)+ts*q1p(k);  % angular position q1 in radians (rad)
        q2(k+1)=q2(k)+ts*q2p(k);  % angular position q2 in radians (rad)
        q3(k+1)=q3(k)+ts*q3p(k);  % angular position q3 in radians (rad)
        
       % Geometric Model of Mobile platform
        xp(k+1)=x1(k+1)+a*cos(phi(k+1));
        yp(k+1)=y1(k+1)+a*sin(phi(k+1));

        %Geometric Model of Arm
        hx(k+1)=xp(k+1)+l1*cos(q2(k+1))*cos(phi(k+1)+q1(k+1))+l2*cos(q2(k+1)+q3(k+1))*cos(phi(k+1)+q1(k+1));   % initial interest position (x axis) in meters (m)
        hy(k+1)=yp(k+1)+l1*cos(q2(k+1))*sin(phi(k+1)+q1(k+1))+l2*cos(q2(k+1)+q3(k+1))*sin(phi(k+1)+q1(k+1));   % initial interest position (y axis) in meters (m)
        hz(k+1)=b+c+l1*sin(q2(k+1))+l2*sin(q2(k+1)+q3(k+1)); % initial interest position (z axis) in meters (m)m)

end
%%
%%%%%%%%%%%%%%%%%%%%% Stroboscopic movement of the ROBOT %%%%%%%%%%%%%%%%%%%
scene=figure;  % new figure
set(scene,'Color','white');
set(gca,'FontWeight','bold') ;
sizeScreen=get(0,'ScreenSize');
set(scene,'position',sizeScreen); % position and size figure in the screen
axis equal; % Set axis aspect ratios
axis([-10 10 -10 5 0 2]); % Set axis limits 
view([-30 35]); % orientation figure
grid on; % Display axes grid lines
box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Label axis
camlight left % scene light

MobileRobot; % Parameters of platform robot 
M1=MobilePlot(x1(1),y1(1),phi(1));hold on % Plot robot in initial position x1,y1 and phi orientation
M2=plot3(hx(1),hy(1),hz(1),'r','LineWidth',2); % Initial point.
M3=plot3(hxd,hyd,hzd,'g','LineWidth',2); % Initial trajectory.
Arm_Parameters(1); % Parameters of arm robot
M4=Arm_Plot3D(xp(1),yp(1),b,0,0,phi(1)+q1(1),q2(1),q3(1),0);% Plot robot in initial position hx,hy and phi orientation

step=15; % step for simulation

for i=1:step:length(t) % Loop emulation
     delete (M1) % delete trajectory
     delete (M2) % delete platform robot
     delete (M4) % delete arm robot
     
     M1=MobilePlot(x1(i),y1(i),phi(i)); % Plot platform robot again
     M2=plot3(hx(1:i),hy(1:i),hz(1:i),'r','LineWidth',2); % plot trajectory.
     M4=Arm_Plot3D(xp(i),yp(i),b,0,0,phi(i)+q1(i),q2(i),q3(i),0);% Plot arm robot again
     pause(ts) % sample time
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot control errors %%%%%%%%%%%%%%%%%%%%%%%%

errorFig=figure; % new figure
set(errorFig,'position',sizeScreen);

subplot(311)
plot(t,hxe,'r','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('Error hx (m)');
subplot(312)
plot(t,hye,'b','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('Error hy (m)');
subplot(313)
plot(t,hze,'g','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('Error hz (m)');

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot control actions arm velocity %%%%%%%%%%%%%%%%%%%%%%%%

armFig=figure; % new figure
set(armFig,'position',sizeScreen);
subplot(311)
plot(t,q1p,'r','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('angular velocity q1 (rad/s)');
title("Arm velocity")
subplot(312)
plot(t,q2p,'b','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('angular velocity q2 (rad/s)');
subplot(313)
plot(t,q3p,'g','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('angular velocity q3 (rad/s)');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot control actions unicycle mobile velocity %%%%%%%%%%%%%%%%%%%%%%%%

uniFig=figure; % new figure
set(uniFig,'position',sizeScreen);
subplot(211)
plot(t,u,'r','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('linear velocity (m/s)');
title("Unicycle mobile velocity")
subplot(212)
plot(t,w,'b','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('angular velocity (rad/s)');
