clc; clear; close all;

%% --- Longitudes de los eslabones (m) ---
L1 = 0.15951;  % Base
L2 = 0.12668;  % Hombro
L3 = 0.12668;  % Codo
L5 = 0.18485;  % Efector final

%% --- Límites articulares realistas (ajustados al diseño físico) ---
q1 = linspace(-pi/2, pi/2, 15);    % Base: ±90°
q2 = linspace(0, 2*pi/3, 15);      % Hombro: 0°–120°
q3 = linspace(-pi/2, pi/3, 15);    % Codo: -90°–60°
q4 = linspace(-pi/3, pi/3, 10);    % Muñeca: ±60°
q5 = linspace(-pi/2, pi/2, 5);     % Rotación efector: ±90°

%% --- Inicialización ---
[X, Y, Z] = deal([]);
R_vectors = [];

%% --- Muestreo del espacio de trabajo ---
for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3)
            for m = 1:length(q4)
                for n = 1:length(q5)
                    % --- Cinemática directa (X positivo) ---
                    tx =  cos(q1(i)) * (L3*sin(q2(j)+q3(k)) + L2*sin(q2(j)) + L5*sin(q2(j)+q3(k)+q4(m)));
                    ty = -sin(q1(i)) * (L3*sin(q2(j)+q3(k)) + L2*sin(q2(j)) + L5*sin(q2(j)+q3(k)+q4(m)));
                    tz =  L1 + L3*cos(q2(j)+q3(k)) + L2*cos(q2(j)) + L5*cos(q2(j)+q3(k)+q4(m));

                    % --- Restricciones físicas ---
                    if tz < L1, continue; end
                    if q2(j) + q3(k) < 0 || q2(j) + q3(k) > pi, continue; end
                    if sqrt(tx^2 + ty^2) < 0.04, continue; end

                    %% --- Guardar punto válido ---
                    X(end+1) = tx;
                    Y(end+1) = ty;
                    Z(end+1) = tz;

                    % --- Orientación simplificada ---
                    Rz1 = [cos(q1(i)) -sin(q1(i)) 0; sin(q1(i)) cos(q1(i)) 0; 0 0 1];
                    Ry2 = [cos(q2(j)) 0 sin(q2(j)); 0 1 0; -sin(q2(j)) 0 cos(q2(j))];
                    Ry3 = [cos(q3(k)) 0 sin(q3(k)); 0 1 0; -sin(q3(k)) 0 cos(q3(k))];
                    Ry4 = [cos(q4(m)) 0 sin(q4(m)); 0 1 0; -sin(q4(m)) 0 cos(q4(m))];
                    Rz5 = [cos(q5(n)) -sin(q5(n)) 0; sin(q5(n)) cos(q5(n)) 0; 0 0 1];
                    R = Rz1*Ry2*Ry3*Ry4*Rz5;

                    R_vectors = [R_vectors; tx ty tz R(1,3) R(2,3) R(3,3)];
                end
            end
        end
    end
end

%% --- Posición home (referencia a 90°) ---
q_home = pi/2;
tx_home =  cos(q_home)*(L3*sin(q_home+q_home) + L2*sin(q_home) + L5*sin(q_home+q_home+q_home));
ty_home = -sin(q_home)*(L3*sin(q_home+q_home) + L2*sin(q_home) + L5*sin(q_home+q_home+q_home));
tz_home =  L1 + L3*cos(q_home+q_home) + L2*cos(q_home) + L5*cos(q_home+q_home+q_home);

%% --- Centrar el espacio de trabajo en el origen (0,0) ---
Xs = X - tx_home;
Ys = Y - ty_home;
Zs = Z;  % Mantiene altura original

% Ajustar vectores de orientación
R_vectors_shift = R_vectors;
R_vectors_shift(:,1) = R_vectors(:,1) - tx_home;
R_vectors_shift(:,2) = R_vectors(:,2) - ty_home;

tx_home_s = 0; % Ahora está en el centro
ty_home_s = 0;

%% --- Gráfica del espacio de trabajo ---
figure('Color','w');
set(gca,'FontName','Georgia');

scatter3(Xs, Ys, Zs, 15, Zs, 'filled'); hold on;
plot3(tx_home_s, ty_home_s, 0, 'r.', 'MarkerSize', 25); % Punto central (no círculo)
text(tx_home_s, ty_home_s, 0, 'Origen', 'FontSize',12, 'Color','k', 'FontName','Georgia');

% Dibujar vectores de orientación (submuestreo)
step = 100;
for idx = 1:step:size(R_vectors_shift,1)
    quiver3(R_vectors_shift(idx,1), R_vectors_shift(idx,2), R_vectors_shift(idx,3), ...
            0.05*R_vectors_shift(idx,4), 0.05*R_vectors_shift(idx,5), 0.05*R_vectors_shift(idx,6), ...
            'Color','k','MaxHeadSize',0.5);
end

axis equal
grid on
xlabel('x [m]','FontSize',12,'FontWeight','bold','FontName','Georgia');
ylabel('y [m]','FontSize',12,'FontWeight','bold','FontName','Georgia');
zlabel('z [m]','FontSize',12,'FontWeight','bold','FontName','Georgia');
title('Espacio de trabajo','FontSize',14,'FontWeight','bold','FontName','Georgia');
view(45,30);
colormap jet;
c = colorbar;
c.Label.String = 'Altura z [m]';
c.FontSize = 12;
c.Label.FontName = 'Georgia';

