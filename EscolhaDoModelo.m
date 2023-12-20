clear all
%dados
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

%Escolha do modelo
disp('1-Maximum vertical velocity');
disp('2-Median vertical velocity');
disp('3-Zero velocity (hover)');
disp('4-Low forward velocity');
mode=input('What is the opperational mode: ');
while mode~=1 && mode~=2 && mode~=3 && mode~=4
    disp('Non avaible option');
mode=input('What is the opperational mode: ');
end


if mode==1
    eixoy=0;
    Thrust=100000;
elseif mode==2
    eixoy=0;
    Thrust=100000/4;
elseif mode==3
    eixoy=0;
    Thrust=g_marte*mt;
elseif mode==4
    eixoy=sin(20*pi/180);
    Thrust=g_marte*mt;
end
run Controlador_H_inf.m
    

% test controlability, observability and stability
[V,DL,W] = eig(A);

if any(real(diag(D)) >=0 ), disp('Linearized system 1 is not stable.'); end

n1_unstable_modes = rank(ctrb(A,B))-12;

if n1_unstable_modes > 0, disp('Linearized system 1 is not controlable.'); end

n1_unobservable_modes = rank(obsv(A,C))-12;
if n1_unobservable_modes > 0, disp('Linearized system 1 is not observable.'); end

