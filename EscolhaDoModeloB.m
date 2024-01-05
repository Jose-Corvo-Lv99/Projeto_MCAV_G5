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
disp('0-Close program');
mode=input('What is the opperational mode: ');
while mode~=1 && mode~=2 && mode~=3 && mode~=4 && mode~=0
    disp('Non avaible option');
mode=input('What is the opperational mode: ');
end


if mode==1
    eixoy=0;
    Thrust=100000;
    run ControladorB_H_inf.m
elseif mode==2
    eixoy=0;
    Thrust=100000/4;
    run ControladorB_H_inf.m
elseif mode==3
    eixoy=0;
    Thrust=g_marte*mt;
    run ControladorB_H_inf.m
elseif mode==4
    eixoy=sin(20*pi/180);
    Thrust=g_marte*mt;
    run ControladorB_H_inf.m
elseif mode==0
    
end