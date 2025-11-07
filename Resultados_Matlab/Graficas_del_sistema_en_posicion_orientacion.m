clear
clc
%% === Configuración global de gráficos ===
set(groot, 'defaultAxesFontName', 'Georgia');       % Fuente de todos los ejes = Arial
set(groot, 'defaultTextFontName', 'Arial');       % Fuente de todos los textos = Arial
set(groot, 'defaultAxesFontSize', 12);            % Tamaño de fuente de los ejes = 12
set(groot, 'defaultLegendFontSize', 11);          % Tamaño de fuente de las leyendas = 11
set(groot, 'defaultLineLineWidth', 1.5);          % Grosor de todas las líneas = 1.5
set(groot, 'defaultFigurePosition', [100 50 800 400]);  % Posición y tamaño de las figuras [left bottom width height]
set(groot, 'defaultAxesGridLineStyle', '-');      % Estilo de la grilla = línea continua
set(groot, 'defaultAxesXLimMode', 'manual');      % Evita que MATLAB cambie automáticamente el límite X
set(groot, 'defaultAxesXLim', [0 30.2]);          % Límite X fijo = 0 a 21.2
set(groot, 'defaultAxesGridAlpha', 0.3);
set(groot, 'defaultAxesGridLineStyle', '--');  % Punteada
set(groot, 'defaultLegendBox', 'on');             % Borde de la leyenda activado
set(groot, 'defaultLegendColor', [1 1 1]);        % Fondo de la leyenda blanco
%% === Leer datos desde CSV ===
%data = readtable('datosx_y_th_Prueba1.csv');
data = readtable('datosx_y_th_Prueba2.csv');
%data = readtable('datosx_y_th_Prueba3.csv');

t = data.sampleTime*10e-2;

%% === Parámetros del robot ===
L1 = 0.15951;
L2 = 0.12668;
L3 = 0.12668;
L5 = 0.18485;
tolerance = 0.1;

n = length(t);

%% =======================
%  Ángulos de Euler (th*m y th)
%% =======================
% Prealocar vectores
alpha_m = zeros(height(data),1);
beta_m  = zeros(height(data),1);
gamma_m = zeros(height(data),1);

alpha_O = zeros(height(data),1);
beta_O  = zeros(height(data),1);
gamma_O = zeros(height(data),1);

for i = 1:height(data)

    % Conjunto th (articulaciones del manipulador)
    th1 = data.th1(i); th2 = data.th2(i);
    th3 = data.th3(i); th4 = data.th4(i); th5 = data.th5(i);

    % === ORIENTACIÓN LOCAL (desde el móvil) ===
    Beta = asin(-0.5*sin(th2 - th1 + th3 + th4) - 0.5*sin(th1 + th2 + th3 + th4));
    Alpha = 0; Gamma = 0;

    if abs(Beta - pi/2) > tolerance && abs(Beta + pi/2) > tolerance
        Alpha = atan2(-(0.5*cos(th1 + th2 + th3 + th4) - 0.5*cos(th2 - th1 + th3 + th4)), ...
                       cos(th2 + th3 + th4));

        Gamma = atan2(-(cos(th1)*cos(th2)*sin(th3)*sin(th4)*sin(th5) ...
                       - cos(th1)*cos(th2)*cos(th3)*cos(th4)*sin(th5) ...
                       - cos(th5)*sin(th1) ...
                       + cos(th1)*cos(th3)*sin(th2)*sin(th4)*sin(th5) ...
                       + cos(th1)*cos(th4)*sin(th2)*sin(th3)*sin(th5)), ...
                       cos(th1)*cos(th2)*cos(th3)*cos(th4)*cos(th5) ...
                       - sin(th1)*sin(th5) ...
                       - cos(th1)*cos(th2)*cos(th5)*sin(th3)*sin(th4) ...
                       - cos(th1)*cos(th3)*cos(th5)*sin(th2)*sin(th4) ...
                       - cos(th1)*cos(th4)*cos(th5)*sin(th2)*sin(th3));
    end

    % Guardar ángulos locales
    alpha_m(i) = rad2deg(Alpha);
    beta_m(i)  = rad2deg(Beta);
    gamma_m(i) = rad2deg(Gamma);

    % === CONVERSIÓN AL SISTEMA GLOBAL ===
    theta = data.th(i); % orientación del móvil (yaw global)

    % Matriz de rotación del móvil (solo yaw θ)
    R_om = [cos(theta) -sin(theta) 0;
            sin(theta)  cos(theta) 0;
                 0           0     1];

    % Matriz local R^m_Pd (convención ZYX)
    Rz = [cos(Gamma) -sin(Gamma) 0;
          sin(Gamma)  cos(Gamma) 0;
               0          0     1];
    Ry = [cos(Beta)  0  sin(Beta);
              0      1     0;
          -sin(Beta) 0  cos(Beta)];
    Rx = [1     0          0;
          0 cos(Alpha) -sin(Alpha);
          0 sin(Alpha)  cos(Alpha)];

    RmPd = Rz * Ry * Rx;

    % Transformación al sistema global
    Ropd = R_om * RmPd;

    % Extraer ángulos globales (ZYX)
    betaO  = asin(-Ropd(3,1));
    gammaO = atan2(Ropd(2,1), Ropd(1,1));
    alphaO = atan2(Ropd(3,2), Ropd(3,3));

    % Guardar globales
    alpha_O(i) = rad2deg(alphaO);
    beta_O(i)  = rad2deg(betaO);
    gamma_O(i) = rad2deg(gammaO);
end
% === EJEMPLO DE GRÁFICA ===
%% === Graficar ángulos en el sistema GLOBAL (O) ===
figure;
plot(t, alpha_O, '-r','LineWidth',1.5); hold on;
plot(t, beta_O,  '-g','LineWidth',1.5);
plot(t, gamma_O, '-b','LineWidth',1.5);
xlabel('Tiempo(s)'); 
ylabel('Orientación del robot(°)');
legend('\alpha_{Arm}^O (deg)','\beta_{Arm}^O (deg)','\gamma_{Arm}^O (deg)');
grid on;

%% === Graficar ángulos en el sistema LOCAL (móvil) ===
figure;
plot(t, alpha_m, '-r','LineWidth',1.5); hold on;
plot(t, beta_m,  '-g','LineWidth',1.5);
plot(t, gamma_m, '-b','LineWidth',1.5);
xlabel('Tiempo(s)'); 
ylabel('Orientación del manipulador(°)');
legend('\alpha_{Arm}^m (deg)','\beta_{Arm}^m (deg)','\gamma_{Arm}^m (deg)');
grid on;

%% =======================
% Posición combinada del efector final absoluto
%% =======================
xm = data.x; ym = data.y; thm = data.th;
th1_data = data.th1; th2_data = data.th2;
th3_data = data.th3; th4_data = data.th4; th5_data = data.th5;

px_abs = zeros(n,1); py_abs = zeros(n,1); pz_abs = zeros(n,1);

for i = 1:n
    th1m = th1_data(i); th2m = th2_data(i);
    th3m = th3_data(i); th4m = th4_data(i);

    term = L3*sin(th2m+th3m) + L2*sin(th2m) + L5*sin(th2m+th3m+th4m);
        px_rel = -cos(th1m) * term;
        py_rel = -sin(th1m) * term;
        pz_rel = L1 + L3*cos(th2m+th3m) + L2*cos(th2m) + L5*cos(th2m+th3m+th4m);

        px_abs(i) = xm(i) + cos(thm(i))*px_rel - sin(thm(i))*py_rel;
        py_abs(i) = ym(i) + sin(thm(i))*px_rel + cos(thm(i))*py_rel;
        pz_abs(i) = pz_rel;
end

%% === Graficar posiciones del móvil y efector absoluto ===
figure;
plot(t, xm, ':r','LineWidth',1.5); hold on;   % móvil x punteado
plot(t, ym, ':g','LineWidth',1.5);           % móvil y punteado
plot(t, px_abs, '-r','LineWidth',1.5);       % efector x continua
plot(t, py_abs, '-g','LineWidth',1.5);       % efector y continua
xlabel('Tiempo(s)'); ylabel('Posición del robot y móvil(m)');
legend('x_m^O (m)','y_m^O (m)','tx_{Arm}^O (m)','ty_{Arm}^O (m)');
grid on;

%% === Graficar orientación del móvil (thm) en grados en ventana aparte ===
figure;
plot(t, rad2deg(thm), '-b','LineWidth',1.5);
xlabel('Tiempo(s)'); ylabel('Orientación  del móvil(°)');
legend('\gamma_m^O (deg)');
grid on;

%% === Quinta gráfica: posición absoluta del efector final en XYZ ===
figure;
plot(t, px_abs, '-r','LineWidth',1.5); hold on;
plot(t, py_abs, '-g','LineWidth',1.5);
plot(t, pz_abs, '-b','LineWidth',1.5);
xlabel('Tiempo(s)'); ylabel('Posición del robot(m)');
legend('tx_{Arm}^O (m)','ty_{Arm}^O (m)','tz_{Arm}^O (m)');
grid on;

%% === Animación 3D del efector final ===
%% === Cargar datos ===
%data = readtable('datosx_y_th_Prueba1.csv');
data = readtable('datosx_y_th_Prueba2.csv');
%data = readtable('datosx_y_th_Prueba3.csv');
t = data.sampleTime * 10e-2;

% Longitudes del robot
L1 = 0.15951;
L2 = 0.12668;
L3 = 0.12668;
L5 = 0.18485;

n = length(t);

xm = data.x; ym = data.y; thm = data.th;
th1_data = data.th1; th2_data = data.th2;
th3_data = data.th3; th4_data = data.th4;

px_abs = zeros(n,1); py_abs = zeros(n,1); pz_abs = zeros(n,1);

for i = 1:n
    th1m = th1_data(i); th2m = th2_data(i);
    th3m = th3_data(i); th4m = th4_data(i);

    term = L3*sin(th2m+th3m) + L2*sin(th2m) + L5*sin(th2m+th3m+th4m);
        px_rel = -cos(th1m) * term;
        py_rel = -sin(th1m) * term;
        pz_rel = L1 + L3*cos(th2m+th3m) + L2*cos(th2m) + L5*cos(th2m+th3m+th4m);

        px_abs(i) = xm(i) + cos(thm(i))*px_rel - sin(thm(i))*py_rel;
        py_abs(i) = ym(i) + sin(thm(i))*px_rel + cos(thm(i))*py_rel;
        pz_abs(i) = pz_rel;
end

%% === Animación 3D con ejes reducidos ===
%% === Animación 3D con escala automática y tiempo real ===
figure;
set(gcf,'WindowState','maximized');   % ventana completa

hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Trayectoria 3D del efector final');
view(45,30);   % Cámara en ángulo 3D


% Escala automática con 10% de margen
margin = 0.1;
xlim([min(px_abs)-margin, max(px_abs)+margin]);
ylim([min(py_abs)-margin, max(py_abs)+margin]);
zlim([min(pz_abs)-margin, max(pz_abs)+margin]);

% Dibujar trayectoria completa en gris
plot3(px_abs, py_abs, pz_abs, '--k','LineWidth',1);

% Punto animado del efector
h_eff = plot3(px_abs(1), py_abs(1), pz_abs(1), ...
    'ro','MarkerFaceColor','r','MarkerSize',6);

% Punto animado de la base móvil
h_base = plot3(xm(1), ym(1), 0, ...
    'bo','MarkerFaceColor','b','MarkerSize',6);

% Animación usando el tiempo real del CSV
for i = 1:n
    % Actualizar efector
    set(h_eff,'XData',px_abs(i),'YData',py_abs(i),'ZData',pz_abs(i));
    % Actualizar móvil
    set(h_base,'XData',xm(i),'YData',ym(i),'ZData',0);
    drawnow;
    
    if i < n
        pause(t(i+1) - t(i));  % diferencia entre tiempos reales
    end
end


