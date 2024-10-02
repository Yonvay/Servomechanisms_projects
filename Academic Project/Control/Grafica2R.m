% Configuración de la comunicación serial
global s;
port = 'COM7'; 
baudRate = 9600; 
s = serialport(port, baudRate); 

% Longitudes de los eslabones
l1 = 20; 
l2 = 20; 

% Configuración de la gráfica
figureHandle = figure('Name', 'Mecanismo 2R Animado', 'NumberTitle', 'off'); % Crear la figura para la gráfica
hold on;
axis equal;
xlim([-40 40]);
ylim([0 40]);
grid on;
title('Mecanismo 2R');
xlabel('X');
ylabel('Y');

t_values = linspace(0, 2*pi, 60);
x = 1*1.46*(cos(3 * t_values + pi/6) + 5) .* cos(t_values) + 13;
y = 1*1.46*(cos(3 * t_values + pi/6) + 5) .* sin(t_values) + 24.5;

% Almacenar las coordenadas de los puntos extremos
extreme_points_x = [];
extreme_points_y = [];

% Crear una nueva figura para los controles de interfaz
controlFig = figure('Name', 'Control de Valores', 'NumberTitle', 'off', 'Position', [100, 100, 300, 250]);

uicontrol('Style', 'text', 'String', 'Petalos (3 - 10):', 'Position', [10, 190, 90, 30], 'Parent', controlFig);
PetalosInput = uicontrol('Style', 'edit', 'Position', [100, 195, 100, 30], 'Parent', controlFig);

uicontrol('Style', 'text', 'String', 'Escala (1 - 1.33):', 'Position', [10, 140, 90, 30], 'Parent', controlFig);
EscalaInput = uicontrol('Style', 'edit', 'Position', [100, 145, 100, 30], 'Parent', controlFig);

uicontrol('Style', 'text', 'String', 'Rotación (°):', 'Position', [10, 90, 90, 30], 'Parent', controlFig);
RotacionInput = uicontrol('Style', 'edit', 'Position', [100, 95, 100, 30], 'Parent', controlFig);

sendButton = uicontrol('Style', 'pushbutton', 'String', 'Iniciar', 'Position', [100, 40, 100, 40], ...
    'Parent', controlFig, 'Callback', @(src, event) sendJson(PetalosInput, EscalaInput, RotacionInput));

isRunning = true; 

while isRunning
    if s.NumBytesAvailable > 0
        % Leer la línea enviada por Arduino
        data = readline(s);

        % Decodificar el JSON
        try
            jsonData = jsondecode(data);
            theta1 = jsonData.a; % Obtener el valor del ángulo 'a'
            theta2 = jsonData.b; % Obtener el valor del ángulo 'b'
            
           
            theta1_rad = deg2rad(theta1);
            theta2_rad = deg2rad(theta2);

            % Cálculo de las posiciones de los puntos de articulación
            x0 = 0; y0 = 0; % Base del mecanismo
            x1 = l1 * cos(theta1_rad);
            y1 = l1 * sin(theta1_rad);
            x2 = l1*cos(theta1_rad) + l2*cos(theta1_rad)*cos(theta2_rad) - l2*sin(theta1_rad)*sin(theta2_rad);
            y2 = l1*sin(theta1_rad) + l2*cos(theta1_rad)*sin(theta2_rad) + l2*cos(theta2_rad)*sin(theta1_rad);

            % Guardar las coordenadas del punto extremo
            extreme_points_x = [extreme_points_x, x2];
            extreme_points_y = [extreme_points_y, y2];

            % Seleccionar la figura principal para graficar
            figure(figureHandle);
            
            % Graficar el mecanismo
            cla; % Limpiar la gráfica para actualizar el mecanismo
            plot([x0, x1], [y0, y1], 'ro-', 'LineWidth', 2); % Eslabón 1
            plot([x1, x2], [y1, y2], 'bo-', 'LineWidth', 2); % Eslabón 2
            plot(x, y);

            % Graficar todos los puntos extremos acumulados como puntos (pixeles)
            plot(extreme_points_x, extreme_points_y, 'g.', 'MarkerSize', 10); % Puntos extremos acumulados como puntos

            drawnow; % Actualizar la gráfica
        catch
            disp('Error al decodificar JSON o actualizar la gráfica');
        end
    end
    pause(0.1); % Pequeña pausa para evitar sobrecarga en la lectura serial
end

% Función de devolución de llamada para enviar JSON
function sendJson(AInput, BInput, CInput)
    global s;
    Petalos = str2double(get(AInput, 'String')); 
    Escala = str2double(get(BInput, 'String'));
    Rotacion = str2double(get(CInput, 'String')); 
    
    if isnan(Petalos) || isnan(Escala) || isnan(Rotacion)
        disp('Error: Por favor ingrese valores numéricos válidos.');
        return;
    end
    
    jsonString = sprintf('{"ini":1, "Petalos":%.2f, "Escala":%.2f, "Rotacion":%.2f}', Petalos, Escala, Rotacion);
    writeline(s, jsonString); % Enviar el JSON por comunicación serial
    disp(['Enviado JSON: ', jsonString]);
end
