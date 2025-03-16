classdef MultiOrientationViewer < handle
    properties
        Figure
        Axes
        Cubes
        Arrows
        FilterNames
        DecimationFactor
        Colors
        OriginalVertices  % Nueva propiedad para almacenar vértices originales
        CubeCenters      % Nueva propiedad para almacenar centros
    end
    
    methods
        function obj = MultiOrientationViewer(numFilters)
            % Incrementar el número de cubos para incluir el ground truth
            totalFilters = numFilters; % Incluye el ground truth
            obj.Figure = figure('Name', 'Multiple Orientation Viewer', ...
                              'Position', [100, 100, 1200, 800], ...
                              'DoubleBuffer', 'on');  % Mejorar rendimiento
            obj.Axes = axes('Parent', obj.Figure);
            hold(obj.Axes, 'on');
            grid(obj.Axes, 'on');
            view(obj.Axes, 3);
            
            % Configurar vista
            axis(obj.Axes, 'equal');
            xlim(obj.Axes, [-6 6]);
            ylim(obj.Axes, [-10 2]);
            zlim(obj.Axes, [-6 6]);
            
            % Agregar etiqueta para el ground truth
            obj.FilterNames = {'Matlab', 'EKF', 'Gyro', 'KF', 'UDU', 'Takasu', 'Carlson', 'Takasu Mejorado', 'Ground Truth'};
            obj.Colors = [0.5 0.5 0.5; 1 0 0; 0 0 1; 0 1 0; 1 0 1; 0 1 1; 1 1 0; 0.8 0.4 0; 0.2 0.6 0.8]; % Color único para el ground truth
            
            % Inicializar contenedores
            obj.Cubes = cell(totalFilters, 1);
            obj.Arrows = cell(totalFilters, 3);
            obj.OriginalVertices = cell(totalFilters, 1);
            obj.CubeCenters = zeros(totalFilters, 3);
            
            % Calcular posiciones
            spacing = 4;
            positions = zeros(totalFilters, 3);
            for i = 1:totalFilters
                row = ceil(i/3);
                col = mod(i-1, 3);
                positions(i,:) = [(col-1)*spacing, (1-row)*spacing, 0];
            end
            
            % Crear objetos una sola vez
            [vertices_base, faces] = obj.createCube(1.0);
            
            % Crear cubos y flechas
            for i = 1:totalFilters
                % Crear cubo
                vertices = vertices_base + positions(i,:);
                obj.OriginalVertices{i} = vertices;  % Guardar vértices originales
                obj.CubeCenters(i,:) = positions(i,:);  % Guardar centro
                
                obj.Cubes{i} = patch('Vertices', vertices, 'Faces', faces, ...
                    'FaceColor', 'flat', 'FaceVertexCData', obj.createCubeColors(), ...
                    'EdgeColor', 'k', 'LineWidth', 1);
                
                % Crear flechas
                arrowLength = 1.4;
                origin = positions(i,:);
                obj.Arrows{i,1} = quiver3(origin(1), origin(2), origin(3), ...
                    arrowLength, 0, 0, 'r', 'LineWidth', 2);
                obj.Arrows{i,2} = quiver3(origin(1), origin(2), origin(3), ...
                    0, arrowLength, 0, 'g', 'LineWidth', 2);
                obj.Arrows{i,3} = quiver3(origin(1), origin(2), origin(3), ...
                    0, 0, arrowLength, 'b', 'LineWidth', 2);
                
                % Añadir etiqueta
                text(positions(i,1), positions(i,2)-1.6, positions(i,3), ...
                    obj.FilterNames{i}, 'HorizontalAlignment', 'center', ...
                    'FontSize', 12);
            end
            
            obj.DecimationFactor = 20;  % Aumentado para mejor rendimiento
        end
        
        function update(obj, quaternions)
            for i = 1:length(obj.Cubes)
                if i <= numel(quaternions)
                    % Obtener matriz de rotación
                    R = quat2rotm(compact(quaternions(i)));
                    
                    % Usar vértices originales almacenados
                    vertices_orig = obj.OriginalVertices{i};
                    center = obj.CubeCenters(i,:);
                    
                    % Aplicar rotación
                    vertices = (R * (vertices_orig - center)')' + center;
                    
                    % Actualizar cubo
                    set(obj.Cubes{i}, 'Vertices', vertices);
                    
                    % Actualizar flechas con menos operaciones
                    for j = 1:3
                        arrow = obj.Arrows{i,j};
                        set(arrow, 'UData', R(1,j) * 1.4, ...
                                 'VData', R(2,j) * 1.4, ...
                                 'WData', R(3,j) * 1.4);
                    end
                end
            end
            drawnow limitrate;  % Limitar la tasa de actualización
        end
    end
    
    methods (Static)
        function [vertices, faces] = createCube(size)
            vertices = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1;
                       -1 -1  1; 1 -1  1; 1 1  1; -1 1  1] * size/2;
            faces = [1 2 6 5; 2 3 7 6; 3 4 8 7;
                    4 1 5 8; 1 2 3 4; 5 6 7 8];
        end
        
        function colors = createCubeColors()
            colors = [1 0 0; 1 0 0;   % X (rojo)
                     0 1 0; 0 1 0;   % Y (verde)
                     0 0 1; 0 0 1];  % Z (azul)
        end
    end
end
