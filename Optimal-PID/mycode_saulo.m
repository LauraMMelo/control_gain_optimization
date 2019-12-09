Kp=0.6254;  % Ganho proporcional 
Ki=0.4577;   % Ganho integral
Kd=0.2187;    % Ganho derivativo 

%idx='IAE';  % Performance index (ISE, IAE, ITSE, ITAE)

nump1=[10]; denp1=[0.1 1];
Gp1=tf(nump1, denp1)

nump2=[1]; denp2=[0.4 1];
Gp2=tf(nump2, denp2)

nump3=[1]; denp3=[1 1];
Gp3=tf(nump3, denp3)

Gp = series(Gp1, series(Gp2,Gp3))

[Y1 T1]=myPID(Kp,Ki,Kd,Gp);  % Call PID controller function 

%Jbefore=objfunc(Y1,T1,idx);  % Compute the performance index

% Plot the step response
figure; plot(T1,Y1,'r-'); grid
xlabel('Time'); ylabel('Amplitude');


[Y2 T2]=myPID_discreto(Kp,Ki,Kd);  % Call PID discreto

figure; plot(T2,Y2,'r-'); grid
xlabel('Time'); ylabel('Amplitude');