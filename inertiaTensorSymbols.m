function I = inertiaTensorSymbols(inputSym, type)
    
    a = inputSym;
    
    switch type
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
           
            I = str2sym([a11, a12, a13;
                         a21, a22, a23;
                         a31, a32, a33]);
                
        case 'diagonal'
            a11 = a + "xx";
            a22 = a + "yy";
            a33 = a + "zz";
            
            I = str2sym([a11, 0, 0;
                         0, a22, 0;
                         0, 0, a33]);
    end        
end

