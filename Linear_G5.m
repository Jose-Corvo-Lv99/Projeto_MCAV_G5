
run Modelo_G5.m


%Linearização

v_x=v(1);
v_y=v(2);
v_z=v(3);
omg_x=omg(1);
omg_y=omg(2);
omg_z=omg(3);

%Para matriz A

A1=zeros(3);
A2=[0,-2*D,-2*D; 
    -2*D,0,-2*D; 
    -2*D,-2*D,0];
A3=skew(g_marte*zI);
A4=[0,v_z,-v_y;
    -v_z,0,v_x;
    v_y,-v_x,0];
A5=zeros(3);%Porque omg_x, omg:y, omg_z são iguais a zero
A6=Euler2Q(lbd);
A7=zeros(3);


A=[   zeros(3), eye(3)    , A1  , zeros(3)
        zeros(3), A2         , A3, A4
        zeros(3), zeros(3)   , A5 , A6
        zeros(3), zeros(3)   , zeros(3)  , A7  ];

%Parar matriz B

B=[ zeros(3,1), zeros(3);
    (1/mt)*zI, zeros(3);
    zeros(3,1), zeros(3);
    zeros(3,1), inv(jlr)];

%Para matriz C
C = [   eye(3)    , zeros(3)   , zeros(3) , zeros(3)
        zeros(1,3), zeros(1,3) , zI'      , zeros(1,3)  ];
%Para matriz D
D = zeros(4);

%Simulação do modelo linear
sys = ss(A,B,C,D);

[Vj,Jor] = jordan(A),
% [Vj,Jor] = jordan(sym(A)),
[V,DL,W] = eig(A);
mode_obs = C*V,
mode_ctrl = W'*B,




