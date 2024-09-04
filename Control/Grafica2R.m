% Configuración de la comunicación serial
global s;
port = 'COM5'; 
baudRate = 9600; 
s = serialport(port, baudRate); 

% Longitudes de los eslabones
l1 = 20; 
l2 = 20; 

% Configuración de la gráfica
figureHandle = figure('Name', 'Mecanismo 2R Animado', 'NumberTitle', 'off'); % Crear la figura para la gráfica
hold on;
axis equal;
xlim([0 60]);
ylim([0 60]);
grid on;
title('Mecanismo 2R');
xlabel('X');
ylabel('Y');


% Almacenar las coordenadas de los puntos extremos
extreme_points_x = [];
extreme_points_y = [];

% Crear una nueva figura para los controles de interfaz
controlFig = figure('Name', 'Control de Valores', 'NumberTitle', 'off', 'Position', [100, 100, 300, 200]);


uicontrol('Style', 'text', 'String', 'Escala:', 'Position', [10, 140, 80, 30], 'Parent', controlFig);
EscalaInput = uicontrol('Style', 'edit', 'Position', [100, 145, 100, 30], 'Parent', controlFig);

uicontrol('Style', 'text', 'String', 'Rotación:', 'Position', [10, 90, 80, 30], 'Parent', controlFig);
RotacionInput = uicontrol('Style', 'edit', 'Position', [100, 95, 100, 30], 'Parent', controlFig);


sendButton = uicontrol('Style', 'pushbutton', 'String', 'Iniciar', 'Position', [100, 40, 100, 40], ...
    'Parent', controlFig, 'Callback', @(src, event) sendJson(EscalaInput, RotacionInput));

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
            x2 = x1 + l2 * cos(theta1_rad + theta2_rad);
            y2 = y1 + l2 * sin(theta1_rad + theta2_rad);

            % Guardar las coordenadas del punto extremo
            extreme_points_x = [extreme_points_x, x2];
            extreme_points_y = [extreme_points_y, y2];

            % Seleccionar la figura principal para graficar
            figure(figureHandle);
            
            % Graficar el mecanismo
            cla; % Limpiar la gráfica para actualizar el mecanismo
            plot([x0, x1], [y0, y1], 'ro-', 'LineWidth', 2); % Eslabón 1
            plot([x1, x2], [y1, y2], 'bo-', 'LineWidth', 2); % Eslabón 2

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
function sendJson(AInput,BInput)
    global s;
    Escala = str2double(get(AInput, 'String')); 
    Rotacion = str2double(get(BInput, 'String')); 
    
    
    if isnan(Escala) || isnan(Rotacion)
        disp('Error: Por favor ingrese valores numéricos válidos para maxA y maxB.');
        return;
    end
    
    jsonString = sprintf('{"ini":1, "Escala":%.2f, "Rotacion":%.2f}', Escala, Rotacion);
    writeline(s, jsonString); % Enviar el JSON por comunicación serial
    disp(['Enviado JSON: ', jsonString]);
end
