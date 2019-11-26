function [Kp_best Ki_best Kd_best]=pso_pid(Gp,Ni,idx)
%
% Search for optimal PID parameters using the global random search (GRS) method
%
% INPUTS: controller gains (Kp,Ki,Kd), plant transfer function (Gp), number 
%         of iterations (Ni) and performance index (idx)
%
% Author: Guilherme Barreto
% Date: 11/09/2018

limites=[0 300];   % Intervalo de busca para cada parametro

M=10;  % tamanho populacao
c1=2; c2=c1;  % constantes de aceleracao

wInit=0.1;   % valor inicial da constante de inercia
wFinal=0.1;  % valor final da constante de inercia
w=wInit;

%Passo 1: Gerar populacao inicial
X=randi(limites,M,3);  % Matriz de posicoes iniciais
V=zeros(size(X)); % Matriz de velocidades iniciais

% Avalia populacao inicial
for i=1:M,
    Kp=X(i,1); Ki=X(i,2); Kd=X(i,3);
    [Y T]=myPID(Kp,Ki,Kd,Gp);
    Jcand(i)=objfunc(Y,T,idx);
end

%% N primeira rodada Xbest = P(0) e Jbest=Jcand
Xbest=X;
Jbest=Jcand;

%% Encontra melhor particula da rodada
[Jg Ig]=min(Jcand); % Indice da melhor particula
Xg=X(Ig,:);  % Ganhos da melhor particula

%% Roda PSO por Ni geracoes
for t=1:Ni,  % Loop da geracao/rodada
  iteracao=t
  for i=1:M,  % Loop do enxame
    % Atualiza velocidades da particula
    deltaV1 = c1*rand*(Xbest(i,:)-X(i,:)); % Termo cognitivo
    deltaV2 = c2*rand*(Xg-X(i,:)); % Termo social
    V(i,:) = w*V(i,:) + deltaV1 + deltaV2;  % atualizacao da velocidade
    X(i,:) = X(i,:) + V(i,:); % atualizacao da posicao
    
    if (sum(X(i,:) < limites(1)) | sum(X(i,:) > limites(2))), % Se estiver fora dos limites,
      %X(i,:) = randi(limites,1,3);  % joga pra dentro aleatoriamente
      X(i,:)=Xg;
    end
    
    % Avalia nova posicao da i-esima particula
    Kp=X(i,1); Ki=X(i,2); Kd=X(i,3);
    [Y T]=myPID(Kp,Ki,Kd,Gp);
    Jcand(i)=objfunc(Y,T,idx);
    
    if Jcand(i) < Jbest(i),
        Xbest(i,:) = X(i,:);  % atualiza melhor posicao da particula
        Jbest(i) = Jcand(i);  % atualiza melhor performance
    end
  end
  
  %% Encontra melhor particula da rodada
  [Jg_rodada(t) Ig_rodada]=min(Jcand); % Indice da melhor particula da rodada
  
  % Atualiza melhor solucao global, se necessario
  if Jg_rodada(t) < Jg,  
      Xg=X(Ig_rodada,:);  % Ganhos da melhor particula da rodada
      Jg=Jg_rodada(t);
  end
   
   % Atualiza o parâmetro de inércia (decaimento linear)
   w = wFinal + (wInit - wFinal)*(Ni - t)/(Ni - 1);
end

% Plota evolucao do indice de performance
figure; plot(Jg_rodada);
xlabel('Iteration'); 
ylabel('Performance index (best solution)');

%% Melhor solucao apos Ni rodadas
Kp_best=Xg(end,1); 
Ki_best=Xg(end,2);
Kd_best=Xg(end,3);
