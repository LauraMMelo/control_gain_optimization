function [Kp_best Ki_best Kd_best]=grs_pid(Kp,Ki,Kd,Ni,idx)
%
% Search for optimal PID parameters using the global random search (GRS) method
%
% INPUTS: controller gains (Kp,Ki,Kd), plant transfer function (Gp), number 
%         of iterations (Ni) and performance index (idx)
%
% Author: Guilherme Barreto
% Date: 11/09/2018

limites=[0 300];   % Intervalo de busca para cada parametro

Kp_best=Kp;   % Valor inicial para Kp
Ki_best=Ki;   % Valor inicial para Ki
Kd_best=Kd;   % Valor inicial para Kd

[Y T]=myPID_discreto(Kp_best,Ki_best,Kd_best);  % Call PID controller function 
Jbest=objfunc(Y,T,idx);  % Compute the performance index

%%% Roda GRS por Ng iteracoes
for t=1:Ni,
    iteracao=t;
    
    Kp_cand=randi(limites);      % Gera solucao candidata para X
    Ki_cand=randi(limites);	% Gera solucao candidata para Y
    Kd_cand=randi(limites);	% Gera solucao candidata para Y

    [Y T]=myPID_discreto(Kp_cand,Ki_cand,Kd_cand);  % Call PID controller function 
    Jcand=objfunc(Y,T,idx);  % Compute the performance index
    
    if Jcand<Jbest,  % If candidate solution is better than current best solution, 
        Kp_best=Kp_cand;   % then candidate parameters become the new best ones.
	      Ki_best=Ki_cand;
        Kd_best=Kd_cand;

	      Jbest=Jcand;
    end

    aptidao(t)=Jbest;
end

figure; plot(aptidao);
xlabel('Iteration'); 
ylabel('Performance index (best solution)');
