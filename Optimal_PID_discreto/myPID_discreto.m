function [Y T]=myPID_discreto(Kp,Ki,Kd)
%
% PID controller for plant model Gp
% Parameters: Kp,Ki,Kd (controller gains)
% Plant transfer function: Gp
%
% Autor: Guilherme Barreto
% Data: 07/09/2018

Ts=0.01;  % Taxa de amostragem
Tsim=4;   % Tempo de simulacao
t=0:Ts:Tsim;  % Instantes de amostragem
h=0.01; % discretization step (Euler method)
L=length(t);
Yref=ones(1,L);  % Entrada degrau discreto

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


% Funcao de transferencia do controlador: Gc(s) = NUM(s)/DEN(s)
%numc=[Kd Kp Ki]; denc=[1 0];  
%Gc=tf(numc,denc); 

%Gcp=series(Gc,Gp); % Funcao de transferencia do ramo direto: Gc(s)*Gp(s)

%Gcl=feedback(Gcp);  % Funcao de transferencia em malha fechada

Y = vs;
T = t; % Comando opcional
