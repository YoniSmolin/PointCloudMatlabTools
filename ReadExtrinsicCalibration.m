function transformation =  ReadExtrinsicCalibration( filePath )
%ReradExtrinsicCalibration reads the transformation matrix from filePath and return an affine3d object

    fileId = fopen(filePath);
    
    try
        line = fgetl(fileId);    
        assert( ~isempty(regexp(line, 'Translation*', 'once')) );    
        T = zeros(3,1);
        for i = 1:3
            line = fgetl(fileId);
            T(i) = sscanf(line, '%f');
        end
        
        line = fgetl(fileId);
        assert( ~isempty(regexp(line, 'Rotation*', 'once')) );    
        R = eye(3);
        for i = 1:3
            line = fgetl(fileId);
            R(i,:) = sscanf(line, '%f, %f, %f');
        end
    catch exception
        fclose(fileId);
        rethrow(exception);
    end
    
    A = eye(4);
    A(1:3,:) = [R, T];
    transformation = affine3d( A' );
    
end

