function [ xyyaw ] = SE2ToComponents( SE2 )
    
    xyyaw = nan(1, 3);
    xyyaw(1:2) = SE2(1:2, 3);
    xyyaw(3) = SO2ToEulerAngles( SE2(1:2, 1:2) );
    
end
