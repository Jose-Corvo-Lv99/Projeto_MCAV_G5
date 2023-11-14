clear all
%Projeto MCAV

%1.Modelo não linear
%1.1.  Modelo de corpo rigido
%P=P;
%v = velocidade
%R = altitude em relação ao referencial(angulos de euler)
%omga

%p_dot = R*v;

%R_dot=R*skew(omg);

%lbd_dot = Euler2Q(lbd)*omg;

%v_dot= -skew(om)*v + g*R'*zI - R*D*R'*v - 1/m*(fp+D);

%om_dot = -inv(J)*skew(om)*J*om + inv(J)*np;



%1.2. Equações de força e de momento

%fpz=cos((20*pi)/180)*Ti; (Ti=T1+T2+T3+T4)
% no eixo xy temos os pares T1=-T3 e T2=-T4 que se vão anular tendo um T=0 no
% plano xy

%fpxz; como temos os retrorockets a fazer uma angulo de 20º com o eixo dos
%Z, quando estamos em estabilidade, o fpxz=0,
%Quando temos de fazer movimento no plano xz, fpxz~=0
%temos de definir um thrust para cada retroRocket (T1, T2, T3, T4)

%Te= 4*T1;(thrust de equilibrio)
%T = m*(g-

%momento angular omg=


%1.3. Gravidade e arrasto em Marte

%gMarte=3.71*[0, 0, 1] m/s

%como descrever o Drag (D)??
%FaMarte= -R*D*R'*v
%D = -2*Pi*ro_marte*v*(-2*v)




-------------------------
%Equações:
v=10;
g=3.71;%m/s
m=1000;%kg
%fp=4*cos((20*pi)/180)*T;%resultado em radianos
beta = 0.148;% 0.5*area*ro_marte
zI = [0;0;1];
D=2*beta*eye(3);
ro_marte=0.020;%ro de marte
%Calculo do tensor de Inercia
P=1:3;
J =-integral(((skew(P))^2)*ro_marte*(P),0,V);%??????????




A = [   zeros(3), eye(3)     , zeros(3)  , zeros(3)
        zeros(3), D         , skew(g*zI) , zeros(3)
        zeros(3), zeros(3)   , zeros(3)  , eye(3)
        zeros(3), zeros(3)   , zeros(3)  , D/2  ];
B = [   zeros(3,1), zeros(3)
        (-1/m)*zI    , zeros(3)
        zeros(3,1), zeros(3)
        zeros(3,1), J^(-1)  ];
C = [   eye(3)    , zeros(3)   , zeros(3) , zeros(3)
        zeros(1,3), zeros(1,3) , zI'      , zeros(1,3)  ];
D = zeros(4);

sys = ss(A,B,C,D);

[Vj,Jor] = jordan(A),
[V,DL,W] = eig(A);
mode_obs = C*V,
mode_ctrl = W'*B,

% simulation parameters
nx = 12;
ny = 4;
x0 = zeros(nx,1);
Dt = 0.01;
t = 0:Dt:10;
u_L = [0.2*m*g;0.01;0.01;0.01]*(t>=0);
Te = m*g; % equilibirum thrust
u_NL = [Te;0;0;0]*ones(size(t)) + u_L;



% simulate linear system:
y_L = lsim(sys,u_L,t,x0)';

% simulate nonlinear system
Nsim = length(t);
x = zeros(nx,Nsim);
y = zeros(ny,Nsim);
x(:,1) = x0;
for k = 1:Nsim
    % prepare variables:
    p   = x(1:3,k);
    v   = x(4:6,k);
    lbd = x(7:9,k);
    omg  = x(10:12,k);
    R = Euler2R(lbd);
    T = u_NL(1,k);
    np = u_NL(2:4,k);
    
    % compute state derivative:
    fp=4*cos((20*pi)/180)*T;
    p_dot = R*v;
    R_dot=R*skew(omg);
    lbd_dot = Euler2Q(lbd)*om;
    v_dot= -skew(omg)*v + g*R'*zI - R*D*R'*v - 1/m*(fp+fa);
    om_dot = -inv(jrl)*skew(omg)*jrl*om + inv(jrl)*np;
    
end