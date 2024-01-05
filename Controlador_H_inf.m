

run Linear_G5.m



% Formulação do sistema:
Gss = ss(A,B,C,D);
Gtf = tf(Gss)
s = tf('s');


%Analise de frequiancia SVD 
%SVD de A
rank(A);
[V,DL,W] = eig(A)

figure(4);
sigmaplot(Gtf);
grid on;

figure(5);
set(gcf,'defaultLineLineWidth',0.5);
pzmap(Gtf);
grid on;


%Controlador LQR
Q = diag([19,10,1,0.1,0.01,0.001,0.0001,0.00001,0.000001,0.0000001,0.00000001,0]);
R = 0.1;
Klqr = lqr(A,B,Q,R);
lbd_CL_lqr = eig(A-B*Klqr); 
if any(real(lbd_CL_lqr) >= 0), disp('CL system with LQR not stable'); else, disp('CL system with LQR is stable'); end

%Desenho do Controlador Hinf
A0 = A;
B1 = zeros(12,4);
B2= B;
W1 = sqrt(Q);
W2 = sqrt(R);
C1 = [W1;zeros(1,12)];
D11 = zeros(6,7);
D12 = [zeros(5,1);W2];
C2 = -eye(12);
D21 = eye(6);
D22 = zeros(6,2);
D31=zeros(13,4);
D32=D31;
B0 = [ B1 B2 ]
C0 = [ C1
       C2 ];
D0 = [D11 D12;
      D21 D22;
      D31 D32];
P = ss(A0,B0,C0,D0);
nmeas = 12;
ncont = 4;

%Controlador Hinf
[Kinf,CLinf,gammainf,info_inf] = hinfsyn(P,nmeas,ncont);

% tests on P:
if (nx - rank(ctrb(A0,B2))) > 0, disp('A1.1 on P: system uncontrolable'); else disp('A1.1 on P: OK'); end
if (nx - rank(obsv(A0,C2))) > 0, disp('A1.2 on P: system unobservable'); else disp('A1.2 on P: OK'); end
if (size(D12,2) - rank(D12)) > 0, disp('A2.1 on P: D12 is column rank deficient'); else disp('A2.1 on P: OK'); end
if (size(D21,1) - rank(D21)) > 0, disp('A2.2 on P: D21 is row rank deficient'); else disp('A2.1 on P: OK'); end
syms w real; 


[K2,CL2,gamma2,info_2] = h2syn(P,nmeas,ncont);
poles_CL2 = pole(CL2);
if any(real(poles_CL2) >= 0), disp('CL system with H2 controller not stable'); else, disp('CL system with H2 controller is stable'); end

[Kinf,CLinf,gammainf,info_inf] = hinfsyn(P,nmeas,ncont);
poles_CLinf = pole(CLinf);
if any(real(poles_CLinf) >= 0), disp('CL system with Hinf controller not stable'); else, disp('CL system with Hinf controller is stable'); end


figure(6);
sigma(Gtf,Kinf,CLinf);
grid on;
legend('Gtf','Kinf','CLinf');


i_ctr = 1; % Hinf controller

%Simulação do Controlador
Dt = 0.01;
t = 0:Dt:60;
r = [p_dot(3)*(t>=0);v(3)*(t>=0);lbd(3)*(t>=0);omg(3)*(t>=0)];
NSim = length(t);
nx = 12;
x = zeros(nx,NSim);
u = zeros(4,NSim);
x(:,1) =x0;
for k = 1:NSim

    % get measurements:
    y(:,k) = C*x(:,k);

    % get control action:
    %switch i_ctr
      % case 1 % Hinf controller
            v = -[(r(:,k)-y(:,k));-x(5:nx,k)];
            u(:,k) = Kinf.C*v; % approximation for LQR-like performance of Hinf

       
    %end

    % simulate system:
    x_dot = A*x(:,k) + B*u(:,k); % system derivatives
    xp = x(:,k) + Dt*x_dot; % integrate system state
    if k < NSim
        x(:,k+1) = xp;
    end

end

figure(7);
plot(t,u);
grid on;
xlabel('$$t [s]$$');
ylabel('$$u(t)$$');
legend('$$u_1$$','$$u_2$$');

figure(8);
plot(t,y,t,r);
grid on;
xlabel('$$t [s]$$');
legend('$$y_1$$','$$r_1$$');
  
run EscolhaDoModelo

