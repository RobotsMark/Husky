function heuristic_value=heuristic(a, b)
    heuristic_value=norm(a-b);
    %heuristic_value=max(abs(a(1)-b(1)), abs(a(2)-b(2)));
    %heuristic_value=1;
end