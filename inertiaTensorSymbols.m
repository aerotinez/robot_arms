function MoI = inertiaTensorSymbols(inputSym, type)
    %%  Moment of Inertia Tensor
    %   This function provides a quick utility for symbolic calculations by
    %   creating inertia tensors using the xx...zz subscripts instead of
    %   the MATLAB default 11...33. Enter the desired symbol representing
    %   the inertia tensor into <inputSym> and select either 'general' or
    %   'diagonal'. The output is a symbolic inertia tensor object.
    a = inputSym;
    
    switch type
        %%  Generalised Moment of Inertia Tensor
        %   Use this mode to obtain a tensor containing both the moments
        %   and products of inertia.
        case 'general'
            a11 = a + "xx";
            a12 = a + "xy";
            a13 = a + "xz";
            a21 = a + "yx";
            a22 = a + "yy";
            a23 = a + "yz";
            a31 = a + "zx";
            a32 = a + "zy";
            a33 = a + "zz";
           
            MoI = str2sym([a11, a12, a13; a21, a22, a23; a31, a32, a33]);
                
        case 'diagonal'
            %%  Diagonal Moment of Inertia Tensor
            %   Use this mode to obtain a diagonal (symmetric) tensor
            a11 = a + "xx";
            a22 = a + "yy";
            a33 = a + "zz";
            
            MoI = str2sym([a11, 0, 0; 0, a22, 0; 0, 0, a33]);
    end        
end

