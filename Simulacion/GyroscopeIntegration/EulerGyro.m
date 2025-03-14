function [phi theta psi] = EulerGyro(p, q, r, dt,phi_i, theta_i,psi_i)
    persistent prevPhi prevTheta prevPsi
    if isempty(prevPhi)
        prevPhi = phi_i;
        prevTheta = theta_i;
        prevPsi = psi_i;
    end
    
    sinPhi = sin(prevPhi); 
    cosPhi = cos(prevPhi);
    cosTheta = cos(prevTheta); 
    tanTheta = tan(prevTheta);
    
    % Calcular ángulos de Euler usando integración
    phi = prevPhi + dt*( p + q*sinPhi*tanTheta + r*cosPhi*tanTheta );
    theta = prevTheta + dt*( q*cosPhi - r*sinPhi );
    psi = prevPsi + dt*( q*sinPhi/cosTheta + r*cosPhi/cosTheta );
    
    % Convertir de XYZ a ZYX usando matrices de rotación
    % Crear matriz de rotación XYZ
    Rx = [1 0 0;
          0 cos(phi) -sin(phi);
          0 sin(phi) cos(phi)];
      
    Ry = [cos(theta) 0 sin(theta);
          0 1 0;
          -sin(theta) 0 cos(theta)];
      
    Rz = [cos(psi) -sin(psi) 0;
          sin(psi) cos(psi) 0;
          0 0 1];
    
    % Matriz de rotación completa XYZ
    R = Rx * Ry * Rz;
    
    % Convertir matriz de rotación a ángulos de Euler ZYX
    theta = asin(R(1,3));
    phi = atan2(-R(2,3)/cos(theta), R(3,3)/cos(theta));
    psi = atan2(-R(1,2)/cos(theta), R(1,1)/cos(theta));
    
    % Actualizar valores previos
    prevPhi = phi;
    prevTheta = theta;
    prevPsi = psi;
end