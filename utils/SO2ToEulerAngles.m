function yaw = SO2ToEulerAngles( SO2 )
    
    yaw = atan2( SO2(2, 1), SO2(1, 1) );
    
end