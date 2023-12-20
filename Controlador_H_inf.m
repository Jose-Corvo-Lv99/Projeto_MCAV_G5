

run Linear_G5.m



%Controlador H∞


% Formulação do sistema:
Gss = ss(A,B,C,D);
Gtf = tf(Gss)

%Analise de frequiancia SVD 
%SVD de A
rank(A);
[U,S,V]=svd(A);

FFFF=[0.1,1,9.9,20];
for i = 1:length(FFFF)
    s = j*FFFF(i);
    [U,S,V] = svd(A);
end

figure(90411);
sigmaplot(Gtf);
grid on;

figure(90412);
pzmap(Gtf);
grid on;



% define generalized model:
P = [  eye(4) zeros(4)   Gtf;
      -eye(4) eye(4)    -Gtf ]
nmeas = 4;
ncont = 4;

% design H infinity controller
[K,CL,gamma,info] = hinfsyn(P,nmeas,ncont)

%figure(904801);
%sigma(G,K,CL);
%grid on;
%legend('G','K','CL');



  


