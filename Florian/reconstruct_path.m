function total_path=reconstruct_path(cameFrom, start, goal)
    total_path = [goal(1), goal(2)];
    previous_step=[goal(1), goal(2)];
    
    while true
        previous_step = [cameFrom(previous_step(1),previous_step(2),1), cameFrom(previous_step(1),previous_step(2),2)];
        if previous_step(1)==start(1) && previous_step(2)==start(2)
            break
        end
        total_path=[total_path;previous_step];
    end  
end