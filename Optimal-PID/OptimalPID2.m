% Optimal PID design using ISE, IAE, ITSE and ITAE as objective functions
% and random search and PSO as optimization technique
%
% Parameters: M=1kg, K=20N/m, F=1N, b=10N.s/m
% Step response closed loop system
%
% Autor: Guilherme Barreto
% Data: 07/09/2018

clear; clc; close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Valores iniciais dos ganhos do controlador  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kp=0.1;  % Ganho proporcional 
Ki=0.1;   % Ganho integral
Kd=0.1;    % Ganho derivativo 

idx='ITSE';  % Performance index (ISE, IAE, ITSE, ITAE)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plant transfer function %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nump1=[10]; denp1=[0.1 1];
Gp1=tf(nump1, denp1)

nump2=[1]; denp2=[0.4 1];
Gp2=tf(nump2, denp2)

nump3=[1]; denp3=[1 1];
Gp3=tf(nump3, denp3)

Gp = series(Gp1, series(Gp2,Gp3))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Compute and plot PID action BEFORE search %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[Y1 T1]=myPID(Kp,Ki,Kd,Gp);  % Call PID controller function 

Jbefore=objfunc(Y1,T1,idx);  % Compute the performance index

% Plot the step response
figure; plot(T1,Y1,'r-'); grid
xlabel('Time'); ylabel('Amplitude');

disp('Type any key to continue!');
disp('');
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Search for the optimal gains %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ni=50; % number of iterations
%[Kp_opt Ki_opt Kd_opt]=pso_pid(Gp,Ni,idx);
[Kp_opt Ki_opt Kd_opt]=grs_pid(Kp,Ki,Kd,Gp,Ni,idx);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Compute and plot PID  action AFTER search %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[Y2 T2]=myPID(Kp_opt,Ki_opt,Kd_opt,Gp);  % Call PID controller function 

Jafter=objfunc(Y2,T2,idx); % Compute the performance index

params=[Kp_opt Ki_opt Kd_opt];

% Plot the step response
figure; plot(T2,Y2,'r-'); grid
xlabel('Time'); ylabel('Amplitude');

disp("Kp"), disp(Kp_opt);
disp("Ki"), disp(Ki_opt);
disp("Kd"), disp(Kd_opt);
disp("Jbefore"), disp(Jbefore);
disp("Jafter"), disp(Jafter);

