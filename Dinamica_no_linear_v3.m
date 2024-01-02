syms xp y z up v w phi theta psi pp q r
    
    syms Fc Fa Fm T Fg n delta_f
    
    syms CL0 CLa CLd CLq CD0 CDa CDd CM0 CMa CMd CMq Iz Iy Ix theta_p rho D Ctm g m
    
    syms F1 F3 M dF1 dF3 dM %dSp
    
    % Transformation matrix from body angular velocity to Tait-Bryan rates
    W = [ 1, 0,      -sin(theta)         ;
      0, cos(phi),  cos(theta)*sin(phi)  ;
      0, -sin(phi), cos(theta)*cos(phi) ];

    Winv = simplify(inv(W));
  %
    % Rotation matrix from body to world frame, input: roll, pitch, yaw
    R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) ;
     cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) ;
     -sin(theta),       sin(phi)*cos(theta),                      cos(phi)*cos(theta)                     ];

    % Matrix of mass inertia
    I = [Ix             -0.001           7.868*10^(-5)  ;
         -0.001          Iy             -7.185*10^(-5)  ;
         7.868*10^(-5)  -7.185*10^(-5)      Iz];
 
 %Definciones
 
    %Ra = 2*10^(-2);                  % Radio del disco
    Adisk = 0.019;      % Area del disco (m^2)
    Rl = 0.4000; % Wingspan (m)
    e = 0.08; %excentricidad (m)
    vi = sqrt(m*g/(2*rho*pi*Rl^2));    % Velocidad inducida
    lbe = (Rl/2+e);          % radio vector del blade element % Es de otro paper el valor
    vce = [up;v;w] + cross([pp;q;r],[lbe;0;0])+[0;0;vi]; % Vector de la velocidad del blade element
    T = rho*n^2*D^4*Ctm;    % Empuje 
    qb = 0.5*rho*(norm(vce))^2;% Presion dinamica
    a_inc = 0.1919; % angulo de incidencia 11 grados
    a_atk = atan(vce(3,1)/(vce(2,1))); % angulo de ataque Up/Ut
    a_eff = a_inc - a_atk; % angulo de efectivo de ataque
    c = 0.175; % Distancia media del chord en metros 150, 200
    S = 0.0190; % m^2 Area del pala % Es de  otro paper el valor
    CL = CL0 + CLa*a_eff + CLd*delta_f + CLq*q*c/(2*norm(vce)); %Coeficiente aerodinamico de sustentacion
    CD = CD0 + CDa*abs(a_eff) + CDd*abs(delta_f); % Coeficiente aerodinamico de arrastre
    dL =  qb*CL*S;
    dD =  qb*CD*S;
    
   

% Coeficiente aerodinamico del momento en el monocoptero
CM1 = CM0 + CMa*a_eff + CMd*delta_f + CMq*0.5*q*c/(norm(vce));
    
    % Fuerza aerodinamica
    dF1 = dD*cos(a_atk) + dL*sin(a_atk); % dA debe tenderer F1 = 0
    dF3 = dL*cos(a_atk) - dD*sin(a_atk);% dN
    dM = qb*c*CM1*S; % dM
    %dSp = 0.5*rho*norm(vce)*vce(2,1)*CD*S;% Fuerza de respuesta del aleron (valor minimo no considerado)
    
    
    Fa = [-dF1; 0; -dF3];
    Fm = [0;T*cos(theta_p);T*sin(theta_p)];
    
    %Corregir los torques
    
    CA = [0.0118830; lbe+e; 0.0027860];    % posicion del centro aerodinamico
    CM = [-0.203288; -0.00775; -0.0707140]; % posicion del centro de masa del motor
    COM = [0.0108830; 0.0004150; 0.0037860];% posicion del centro de masa del UAV
    
    Sm = CM - COM; % Vector de distancia del centro de masa del motor al centro de masa del cuerpo
    Sa = CA - COM; % Vector de distancia del centro de masa del centro aerodinamico al COM
    
    %Torque aerodinamico
    ta = cross(Sa, Fa) + [0;dM;0];
    
    %Torque del motor
    tm = cross(Sm,Fm);
    
    %Torque en el cuerpo
    tc = ta+tm;
    
    %Fuerzas en el cuerpo
    Fc = Fa + Fm + R'*[0;0;m*g]; %

% Vectores de los estados
nw = [phi theta psi].';     % Orientacion (diagrama inercial)
vc = [up v w].';  % Velocidad lineal (diagrama del cuerpo)
wc = [pp q r].';  % Velocidad angular (diagrama del cuerpo)
pw = [xp y z].';  % Posicion (diagrama inercial)


%Vector total del estado
X = [nw(1);nw(2); vc; wc]; %Se elimina las variables (yaw, x,y,z)
%-Subsistema de altitud y orientacion
X_red = [nw(1);nw(2); vc(3); wc]; % Reduced state vector (only attitude and altitude)
X_hor = [vc(1); vc(2)]; % Reduced state vector for horizontal movements

X_roll = [nw(1); wc(1); vc(1)];

% Input vector for horizontal model
U_hor = [phi; theta];

% Roll 
U_roll = [delta_f];
% Vector de entrada
U = [n; delta_f];

%% Dinamica translacional
pw_dot = R*vc;
vc_dot = 1/m*Fc-cross(m*wc, vc);

%% Dinamica rotacional
nw_dot = Winv*wc;
wc_dot = inv(I)*(tc-cross(wc, I*wc));


% Modelo no-lineal combinado
f = [nw_dot(1);
     nw_dot(2);
      vc_dot;
      wc_dot];
% Reduced non-linear model
f_red = [ nw_dot(1);
          nw_dot(2);
          vc_dot(3);
          wc_dot     ];
      
% Horizontal non-linear model
f_hor = [ vc_dot(1) ;
          vc_dot(2)];
      
      
f_roll = [ nw_dot(1) ;
          wc_dot(1)  ;
          vc_dot(1) ]; 
  
%Usando el metodo del Jacobiano, se calcula la matriz de variables adjuntas al
%sistema

A = jacobian(f, X);
B = jacobian(f, U);
C = jacobian(X, X);
Dsys = jacobian(X, U);

% Reduced model (only z-axis in position)
A2 = jacobian(f_red, X_red);
B2 = jacobian(f_red, U);
C2 = jacobian(X_red, X_red);

% Horizontal model (only x- and y-direction)
A3 = jacobian( f_hor, X_hor );
B3 = jacobian( f_hor, U_hor );


% Single dimension model (roll/x axis)
A4 = jacobian( f_roll, X_roll );
B4 = jacobian( f_roll, U_roll );

% Todos los estados son cero en el punto de suspension
xp = 2.6319*10^(-10); y = 1.4418*10^(-11); z = -50; up = -0.016139; v = -0.023368; w = 0; 
phi = -0.088454; theta = 0.065357; psi = 0; pp = 2.6818; q = 3.6196; r = -40.814;

% Se tiene entradas diferente a cero
delta_f = 0.071578; % angulo del flap
n = -90;% velocidad del motor rps

% initu = [-0.515;0.071578]; [-0.088454;0.065357;0;-0.016139;-0.023368;-0.065357;2.6818;3.6196;-40.814;2.6319*10^(-10);1.4418*10^(-11);-50];



% Constantes monocoptero
CL0 = 0.4;        % -
CLa = 6.8755;    % 1/rad
CLd = 1.17;        % 1/rad
CLq = 5.8;        % 1/rad
CD0 = 0.02;        % -
CDa = 0.063;       % 1/rad
CDd = 0.01;        % 1/rad
CM0 = -0.0950;
% -
CMa = -0.1;        % 1/rad
CMd = -0.1425;     % 1/rad
CMq = -2.5;        % 1/rad
m = 0.4220;        % kg
theta_p = 0.384;   % rad
rho = 1.1;         % kg/m^3
D = 0.12;          % m
Ctm = 0.4;         % -
Ix = 0.04;         % kg*m^2
Iy = 0.03;        % kg*m^2
Iz = 0.07;        % kg*m^2
g = 9.807;         % m/s^2

% Se evaluan las matrices A y B, obteniendo el modelo lineal completo alrededor del punto de sustentancin

A_sys = double(vpa(subs(A), 4))
B_sys = double(vpa(subs(B), 4))
C_sys = double(vpa(subs(C), 4))
D_sys = double(vpa(subs(Dsys), 4))

% Reduced model 
A_red = double(vpa(subs(A2), 4))
B_red = double(vpa(subs(B2), 4))
C_red = double(vpa(subs(C2), 4))
D_red = zeros(6,2);

% Horizontal model
A_hor = double(vpa(subs(A3),4))
B_hor = double(vpa(subs(B3),4))
C_hor = eye(2)
D_hor = zeros(2,2);

%% Open Loop dynamics

sys = ss(A_sys,B_sys,C_sys,D_sys);
sys_red = ss(A_red,B_red,C_red,D_red);
%sys_int = ss(A_int,B_int,C_int, D_int);
sys_hor = ss(A_hor, B_hor, C_hor, D_hor);
%sys_hint = ss(A_hint, B_hint, C_hint, D_hint);

%% Diseo del controlador

% Regla de Bryson
% Max angle of 0.3 radians. Maximum angular rate of 5 rad/second000
% Maximo angulo de 0.3 rad. Maxima tasa angular de 5 rad/s
Q = [ 1/0.5^2     0     0      0      0      0  ;  % Roll
      0        1/0.5^2  0      0      0      0  ;  % Pitch
      0        0        1/1^2  0      0      0  ;  % w
      0        0        0      1/0.555^2  0      0  ;  % p
      0        0        0      0      1/4^2  0  ;  % q
      0        0        0      0      0      1/4^2 ]; %r 
     
Q_red = Q;  

% Accion integral
%Q(9,9) = [ 1/15^2 ]; % z

% Max actuation angle of +-10 degress
%R = [ 1/0.01^2   0       ; % delta_f
%      0        1/1^2  ]; % n

R = [ 1/20^2   0       ; % n
      0        1/0.01^2  ]; % delta_f
      

% Maxima actuacion deQ, R);
K_red = lqr(sys_red, Q_red, R);
K_hov = lqr(sys_red, Q, R);

% Calculo del limite integral que coincide con la velocidad
%del motor en el regimen estacionario
int_lim = n/K_hov(2,6) + n*0.005;

sys_d = c2d(sys_red, 0.008*10^(-3), 'zoh' )

%K_lqrd = dlqr(A_red, B_red, Q, R);
 
 matrix_to_cpp( K_hov )
Ctrb = ctrb(sys_d.A, sys_d.B)
rank_Ctrb = rank(Ctrb)
%ctrb_matrix = ctrb(sys_d, B_sys);

% Calcuate closed loop system
%figure(1)
%cl_sys = ss((A_red - B_red*K_red), B_red, C_red, D_red );
sys_cl_hov = feedback( sys_red*K_red, C_red);
%figure(3)
%step(sys_cl_hov)

figure(1)
pzmap(sys_cl_hov);%pzmap(sys_cl_hov);
[phi,z] = pzmap(sys_cl_hov)%[phi,z] = pzmap(sys_cl_hov)
grid on

Q_pos = [ 1/1^2  0        ;% u
          0        1/1^2 ];% v

Q_hor = Q_pos;      
           
%R_pos = [ 1/0.00000005^2  0;
%          0          1/0.00000005^2];

R_pos = [ 1/0.05^2  0;
          0          1/0.05^2];
%K_pos = lqr(sys_hint, Q_pos, R_pos);
K_hor = lqr(sys_hor, Q_hor, R_pos);
matrix_to_cpp( K_hor )
sys_cl_pos = feedback( sys_hor*K_hor, C_hor);
figure(2)
pzmap(sys_cl_pos);
[phi2,z2] = pzmap(sys_cl_pos)
grid on

%sys_total = series( sys_cl_pos, sys_cl_pos )matrix_to_cpp( K_pos )

% Symbolic Discretization

% syms dt;
% M = expm([A_sym, B_sym; zeros(12,12), zeros(12,2) ]*dt);
% 
% Ad = M(1:12, 1:12);
% Bd = M(1:12, 13:17);

%Roll and pitch Control


%% Functions 

function matrix_to_cpp( matrix)

    name = inputname(1);
    [m, n] = size(matrix);
    
    tol = 1.e-6;
    matrix(matrix<0 & matrix>-tol) = 0;
    
    fprintf('Matrix %s \n', name);
    
    for i = 1:m
        line = string();
        for j = 1:n
            str = sprintf('%.4f,', round( matrix(i,j), 4 ) );
            value = pad(str, 10, 'left');

            line = append( line, value );
        end
        disp(line);
    end
end

