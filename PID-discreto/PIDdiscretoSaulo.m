% Implementacao de um sistema de controle PID discreto
% em malha fechada
%
% Planta discretizada via Representacao Entrada-Saida (IO, Input-Output)
%
% Parametros da planta: M=1kg, K=20N/m, F=1N, b=10N.s/m
%
% Autor: Guilherme Barreto
% Data: 23/10/2018

clear; clc; close all

Ts=0.01;  % Taxa de amostragem
Tsim=2;   % Tempo de simulacao
t=0:Ts:Tsim;  % Instantes de amostragem
h=0.01; % discretization step (Euler method)
L=length(t);
Yref=ones(1,L);  % Entrada degrau discreto

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Valores iniciais dos ganhos do controlador  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kp=0.6254;  % Ganho proporcional 
Ki=0.4577;   % Ganho integral
Kd=0.2187;    % Ganho derivativo 


%%%%%%%%%%%%%%%%%%%%%
%%% Taus e ganhos  %%
%%%%%%%%%%%%%%%%%%%%%

%Ganhos 
Ka=10;  
Ke=1;   
Kg=1;    
Kr=1;

%Taus
tau_a = 0.1;
tau_e = 0.4;
tau_g = 1;
tau_r = 0.01;

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

[Y1 T1]=myPID(Kp,Ki,Kd,Gp, t);  % Call PID controller function 

figure; plot(T1,Y1,'r-'); grid;
xlabel('tempo (segundos)');
ylabel('Amplitude degrau');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Discretizacao da planta %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Condicoes iniciais

alfa_a = tau_a/(tau_a+h);
alfa_e = tau_e/(tau_e+h);
alfa_g = tau_g/(tau_g+h);
alfa_r = tau_r/(tau_r+h);

vs=[0;0];
vr=[0;0];
vf=[0;0];
vt=[0;0]; 
u=[0;0];
erro=[0;0];
somaErro=0;
for k=3:L,
  erro(k) = Yref(k) - vs(k-1);   % Erro no instante k

  somaErro=somaErro+erro(k);  % Integral (acumulador) do erro 
  
	u(k) = Kp*erro(k) + Ki*h*somaErro + (Kd/h)*(erro(k)-erro(k-1));  % Sinal de controle no instante k
  

  
  % Planta discretizada pelo metodo de Euler
  vr(k) = alfa_a*vr(k-1) + Ka*(1-alfa_a)*u(k);
  vf(k) = alfa_e*vf(k-1) + Ke*(1-alfa_e)*vr(k);
  vt(k) = alfa_g*vt(k-1) + Kg*(1-alfa_g)*vf(k);
  vs(k) = alfa_r*vs(k-1) + Kr*(1-alfa_r)*vt(k);
end

figure; plot(t,Yref,'r-',t,vs,'b-'); grid;
xlabel('tempo (segundos)');
ylabel('Amplitude');
title('Resposta ao Degrau Unitario');



