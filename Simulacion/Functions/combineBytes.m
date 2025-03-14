function result = combineBytes(byte1, byte2)
    % Combina los bytes para formar un entero de 16 bits con signo

    % Primero, combinamos los dos bytes en un entero de 16 bits sin signo
    combined = bitor(bitshift(uint16(byte1), 8), uint16(byte2));

    % Convertimos el valor combinado directamente a int16
    result = typecast(uint16(combined), 'int16');
end