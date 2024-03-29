% Implementacao do algoritmo de busca aleatoria LOCAL (LRS, global random search)
% para encontrar o maximo de uma funcao de 2 variaveis
%
%     funcao 1:  f(x,y)=(x-20)^2 + (y-20)^2 + 50;  0 <= x,y <= 40
%     funcao 2:  f(x,y)=60x+100y-1.5*x^2-1.5*y^2-xy, 0 <= x,y <=60;
%
% Autor: Guilherme A. Barreto
% Data: 22/11/2017

clear; clc; close all;

%%% Parametros do AG
Ng=500;   % Numero de iteracoes

dp=0.5;  % desvio-padrao da perturbacao aleatoria

funcao=2;  % Escolhe funcao-objetivo

% Escolhe limites do dominio de x
if funcao==1,
	limites=[0 40];   % Limites do intervalo para funcao: F=x^2
else    
	limites=[0 60];   % Limites do intervalo de busca para funcao: F=x*sin(10*pi*x)+1;
end

x_best=unifrnd(limites(1),limites(2));   % Gera solucao inicial dentro do dominio de X
y_best=unifrnd(limites(1),limites(2));	 % Gera solucao inicial dentro do dominio de Y

Fbest=func_objetivo2D(x_best,y_best,funcao);  % Avalia solucao inicial

Xbest=[x_best y_best];
%%% Roda LRS por Ng iteracoes
for t=1:Ng,
    iteracao=t;
    
    x_cand(t)=x_best + normrnd(0,dp);      % Gera solucao candidata na vizinhanca de x_best
    y_cand(t)=y_best + normrnd(0,dp);      % Gera solucao candidata na vizinhanca de y_best

    Fcand=func_objetivo2D(x_cand(t),y_cand(t),funcao);  % Avalia solucao candidata
    
    if Fcand>Fbest,
        x_best=x_cand(t);    % Se [x_cand y_cand] produzem melhor resultado que [x_best  y_best], viram "melhor solucao ate o momento"
	      y_best=y_cand(t);
        Fbest=Fcand;
    end

    aptidao(t)=Fbest;
    
    Xbest=[Xbest; x_best y_best];
end

[x_best  y_best], Fbest

figure; plot(aptidao);
xlabel('Iteration');
ylabel('Fitness');

figure; plot(x_cand,y_cand,'bo'); hold on;
plot(x_cand(1),y_cand(1),'r*');
plot(Xbest(:,1),Xbest(:,2),'r-');
title('espaco de busca');