


%Parametros de Marte 
g_marte=3.73;%m/s
ro_marte=0.020;%kg/m^3

%Dimensões do rover

xr=3;%metros
yr=2.7;
zr=2.2;
mr=1025;%kg

jr=[(1/12)*mr*(yr^2+zr^2) 0 0
    0 (1/12)*mr*(xr^2+zr^2) 0
    0 0 (1/12)*mr*(yr^2+xr^2)];

%Dimensões do lander

xl=4;%m
yl=3.7;
zl=3.2;
ml=1100;%kg

jl=[(1/12)*ml*(yl^2+zl^2) 0 0
    0 (1/12)*ml*(xl^2+zl^2) 0
    0 0 (1/12)*ml*(yl^2+xl^2)];


jlr=jr+jl;
mt=mr+ml;

Area_l=xl*yl;
Beta=0.5*2.05*ro_marte*Area_l;
D=Beta;

%Definição dos eixos
cm_t=(ml*(zr+zl/2)+mr*(zr/2))/mt;

%O centro do eixo das coordenadas vai estar no centro de massa
pz_cm=cm_t-zr;

%Definição de um thrust
T1=Thrust/4;
T2=T1;
T3=T1;
T4=T1;


%Posições dos retrorockets e purpulsão
%p1=[-xl/2 -yl/2 -pz_cm];
p1=[0 pz_cm -yl/2
    -pz_cm 0 xl/2
    yl/2 -xl/2 0];
Fp1=T1*[sin(20*pi/180); eixoy; cos(20*pi/180)];

np1=p1*Fp1;

%p2=[-xl/2 yl/2 -pz_cm];
p2=[0 pz_cm yl/2
    -pz_cm 0 xl/2
    -yl/2 -xl/2 0];
Fp2=T2*[sin(20*pi/180); eixoy; cos(20*pi/180)];

np2=p2*Fp2;

%p3=[xl/2 -yl/2 -pz_cm];
p3=[0 pz_cm -yl/2
    -pz_cm 0 -xl/2
    yl/2 xl/2 0];
Fp3=T3*[-sin(20*pi/180); eixoy; cos(20*pi/180)];

np3=p3*Fp3;

%p4=[xl/2 yl/2 -pz_cm];
p4=[0 pz_cm yl/2
    -pz_cm 0 -xl/2
    -yl/2 -xl/2 0];
Fp4=T4*[-sin(20*pi/180); eixoy; cos(20*pi/180)];

np4=p4*Fp4;


fp=[Fp1 Fp2 Fp3 Fp4];
T_m=Fp1+Fp2+Fp3+Fp4;
np=[np1; np2; np3; np4];
%Modelo não linear

zI = [0;0;1];

% Parametros de simulação
nx = 12;
ny = 4;
x0 = zeros(nx,1);
Dt = 0.1;
t = 0:Dt:60;
T=mt*g_marte;
u_NL=[T;0;0;0]*ones(size(t));


C = [   eye(3)    , zeros(3)   , zeros(3) , zeros(3)
        zeros(1,3), zeros(1,3) , zI'      , zeros(1,3)  ];

% simulação do sistema não-linear
Nsim = length(t);
x = zeros(nx,Nsim);
y = zeros(ny,Nsim);
x(:,1) = x0;
for k = 1:Nsim
    % prepare variables:
    p=x(1:3,k);
    v= x(4:6,k);
    lbd = x(7:9,k);
    omg  = x(10:12,k);
    R = Euler2R(lbd);
    T = u_NL(1,k);
    np = u_NL(2:4,k);
    
    % compute state derivative:
    fa=4*pi*ro_marte*v.^2;
    p_dot=R*v;
    lbd_dot = Euler2Q(lbd)*omg;
    v_dot= -skew(omg)*v + g_marte*R'*zI - R*D*R'*v - 1/mt*(T_m+fa);
    om_dot = -inv(jlr)*skew(omg)*jlr*omg + inv(jlr)*np;
    
    %R_dot=R*skew(omg); preferivel ser utilizado o lbd_dot
    
    x_dot = [p_dot;v_dot;lbd_dot;om_dot];

     % integrate state
    x(:,k+1) = x(:,k) + Dt*x_dot;

    % compute current output:
    y(:,k) = C*x(:,k);
    
end
figure(1001);
plot(t,u_NL);
grid on;

figure(1002)
plot(t,y(3,:));




