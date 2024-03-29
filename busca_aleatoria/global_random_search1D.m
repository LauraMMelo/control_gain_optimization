% Implementacao do algoritmo de busca aleatoria GLOBAL (GRS, global random search)
% para encontrar o maximo/minimo de funcoes de 1 variavel
%       f(x) = x^2,   0 < x < 31;
%       f(x) = x*sin(10*pi*x)+1,  -1 < x < 2.
%
% Autor: Guilherme A. Barreto
% Data: 29/11/2017

clear; clc; close all;

%%% Parametros do GRS
Ng=500;   % Numero de iteracoes

funcao=2;  % Escolhe funcao-objetivo

% Escolhe limites do dominio de x
if funcao==1,
	limites=[0 31];   % Limites do intervalo para funcao: F=x^2
else    limites=[-1 2];   % Limites do intervalo de busca para funcao: F=x*sin(10*pi*x)+1;
end


%%%%%%%%%%% Grafico da funcao a ser otimizada
x=limites(1):0.01:limites(2);  % Dominio de x discretizado em incrementos de 0.1
f_opt=func_objetivo1D(x,funcao);  % Avalia solucao inicial
%figure; plot(x,f_opt,'k-');
%hold on
%%%%%%%%%%%

x_best=unifrnd(limites(1),limites(2)); 	% Gera solucao inicial dentro do intervalo permitido

Fbest=func_objetivo1D(x_best,funcao);  % Avalia solucao inicial

plot(x_best,Fbest,'k*');  %%% Plota solucao inicial sobre a funcao a ser otimizada

%%% Roda GRS por Ng geracoes
for t=1:Ng,
    iteracao=t;

    x_cand=unifrnd(limites(1),limites(2)); 		% Gera solucao candidata

    Fcand=func_objetivo1D(x_cand,funcao);  % Avalia solucao candidata
    
    if Fcand>Fbest,	% Se x_cand produz melhor resultado que x_best
        x_best=x_cand;  % x_cand vira "melhor solucao ate o momento"
        Fbest=Fcand;
    end

    plot(x_best,Fbest,'ro');  %%% Plota melhor solucao (solucoes intermediarias)

    aptidao(t)=Fbest;

end

x_best, Fbest

%plot(x_best,Fbest,'b^');  %%% Plota melhor solucao
%hold off

figure; plot(aptidao);
xlabel('Iteracao');
ylabel('y=f(x)');
