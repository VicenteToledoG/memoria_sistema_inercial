function result = combineBytesToFloat(byte1, byte2, byte3, byte4)
    % Combina cuatro bytes para formar un número de punto flotante de 32 bits (float)

    % Primero, combinamos los cuatro bytes en un entero de 32 bits sin signo
    combined = bitor(bitshift(uint32(byte1), 24), ...
             bitor(bitshift(uint32(byte2), 16), ...
             bitor(bitshift(uint32(byte3), 8), uint32(byte4))));

    % Convertimos el valor combinado a un float
    result = typecast(uint32(combined), 'single');
end