clear
close all
clc

%%
% Tiempo de simulacion
tf = 15;
ts = 0.01; %tiempo de muestreo
t = 0:ts:tf;
Q = length(t);

%%%%%%%%%%%%%%%%%%%%%%%% PARAMETROS DEL SISTEMA %%%%%%%%%%%%%%%%%%%%%%%%

hx =  zeros(1,Q+1); %posicion inicial en x en metros [m]
hy =  zeros(1,Q+1); %posicion inicial en y en metros [m]
phi = zeros(1,Q+1); %orientacion inicial en radianes [rad]

%%%%%%%%%%%%%%%%%%%%%%%% POSICION DESEADA %%%%%%%%%%%%%%%%%%%%%%%%

hxd = 1.5; %posicion deseada en x en metros [m]
hyd = 0.9; %posicion deseada en y en metros [m]
phid = -pi/4; %orientacion deseada en radianes [rad]

uMeas = zeros(1,Q); %velocidad maxima [m/s]
wMeas = zeros(1,Q); %velocidad maxima [rad/s]

uRef = zeros(1,Q); %velocidad maxima [m/s]
wRef = zeros(1,Q); %velocidad maxima [rad/s]

l = zeros(1,Q);
rho = zeros(1,Q);
theta = zeros(1,Q);

tiempo = zeros(1,Q);

COM = 'COM15'; %Puerto COM del Arduino

%%%%%%%%%%%%%%%%%%%%%%%% COMUNICACION MATLAB %%%%%%%%%%%%%%%%%%%%%%%%
delete(instrfind({'Port'}, {COM}));
serialPort = serial(COM,'BaudRate',9600);
fopen(serialPort);
pause(3);

disp('Iniciating communication with Arduino...');

for k = 1:Q
    tic

    %%%%%%%%%%%%%%%%%%%%%%%% CONTROL %%%%%%%%%%%%%%%%%%%%%%%%
    % calculo de errores
    l(k)=sqrt((hxd-hx(k))^2+(hyd-hy(k))^2);
    rho(k)=atan2((hyd-hy(k)),(hxd-hx(k)))-phi(k);
    theta(k)=rho(k)+phi(k)-phid;

    % control de parametros
    K1 = 0.1 + (0.5*(k/Q));
    K2 = 0.1 + (0.8*(k/Q));

    %ley de control
    uRef(k) = K1*l(k)*cos(rho(k));
    wRef(k) = K2*rho(k)+(K1/rho(k))*cos(rho(k))*sin(rho(k))*(rho(k)+theta(k));
    
    %%%%%%%%%%%%%%%%%%%% MATLAB %%%%%%%%%%%%%%%%%%%%%%%%%

    trama=[num2str(round(uRef(k),2)) ',' num2str(round(wRef(k),2))]; % String data 

     fprintf(serialPort,'%s\n',trama,'sync'); % send data (String)

     uMeas(k) = fscanf(serialPort, '%f\n'); % receive data (u)
     wMeas(k) = fscanf(serialPort, '%f\n'); % receive data (w)

    

      % a) Simulated robot (Kinematic Model)
      phi(k+1)=phi(k)+ts*wMeas(k);   % real orientation in radians
      hxp=uMeas(k)*cos(phi(k+1)); % real velocity in meters/seconds
      hyp=uMeas(k)*sin(phi(k+1)); % real velocity in meters/seconds

      % b) numerical integral (Euler Method)
      hx(k+1)=hx(k)+ts*hxp;  % real center position (x axis) in meters (m)
      hy(k+1)=hy(k)+ts*hyp;  % real center position (y axis) in meters (m)

      tiempo(k)=toc;

    while toc<ts
    end
end

%%%%%%%%%%%%% Close port MATLAB %%%%%%%%%%%%%%%%%%%%%%

fprintf(serialPort,'%s\n','0,0','sync');
fclose(serialPort);

%%%%%%%%%%%%%%%%%%%%% Stroboscopic movement of the ROBOT %%%%%%%%%%%%%%%%%%%
scene=figure;  % new figure
tam=get(0,'ScreenSize');
set(scene,'position',[tam(1) tam(2) tam(3) tam(4)]); % position and size figure in the screen
axis equal; % Set axis aspect ratios
axis([-0.5 2 -0.5 2 0 0.4]); % Set axis limits 
view([-90 40]); % orientation figure
grid on; % Display axes grid lines
scale = 1;


MobileRobot; % Parameters of robot 
M1=MobilePlot(hx(1),hy(1),phi(1),scale); % Plot robot in initial position x1,y1 and phi orientation
hold on
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Label axis
camlight right % scene light

M2=plot3(hx(1),hy(1),0.06*scale,'go','LineWidth',2); % Initial point.
M3=plot3(hx(1),hy(1),zeros(1),'b','LineWidth',2); % Initial trajectory.
M4=plot3(hxd,hyd,0.06*scale,'ro','LineWidth',2); % Initial point.

step=1; % step for simulation

for i=1:step:k % Loop emulation
     delete (M1) % delete trajectory
     delete (M3) % delete robot
     M1=MobilePlot(hx(i),hy(i),phi(i),scale); hold on; % Plot of robot again
     M3=plot3(hx(1:i),hy(1:i),zeros(1,i),'b','LineWidth',2); % plot trajectory.
    pause(ts) % sample time
end

scene1=figure;  % new figure
set(scene1,'position',[tam(1) tam(2) tam(3) tam(4)]); % position and size figure in the screen
plot(tiempo,'r','LineWidth',2), grid on, xlabel('Samples'); ylabel('Time (s)'); % Label axis

%%%%%%%%%%%%%%%%%%%% Control errors and control actions %%%%%%%%%%%%%%%%%%%
scene2=figure;  % new figure
set(scene2,'position',[tam(1) tam(2) tam(3) tam(4)]); % position and size figure in the screen
subplot(311)
plot(t,l,'r','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('Error l (m)'); % Label axis
subplot(312)
plot(t,rho,'b','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('Error rho (rad)');
subplot(313)
plot(t,thetha,'b','LineWidth',2), grid on, xlabel('Time (s)'); ylabel('Error tehetha (rad)');

scene3=figure;  % new figure
set(scene3,'position',[tam(1) tam(2) tam(3) tam(4)]); % position and size figure in the screen
subplot(211)
plot(t,uMeas,'b','LineWidth',2),hold on, plot(t,uRef,'r','LineWidth',2),grid on, legend('Measurement','Reference'), xlabel('Time (s)'); ylabel('linear velocity (m/s)'); % Label axis
subplot(212)
plot(t,wMeas,'b','LineWidth',2),hold on, plot(t,wRef,'r','LineWidth',2),grid on,legend('Measurement','Reference'), xlabel('Time (s)'); ylabel('angular velocity (rad/s)');

