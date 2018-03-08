function [left_x, left_y, right_x, right_y] = locate_red_t(left_image, right_image,target_fig)

left_x = [];
right_x = [];
left_y = [];
right_y = [];

[BW_left, ~] = createMask(left_image);
[BW_right, ~] = createMask(right_image);

props_left = regionprops(BW_left, 'all');

thresh = 120;
for j = 1:size(props_left, 1)
    if(props_left(j).Area < thresh || ...
            ( ((abs(props_left(j).Orientation) <= 90) && ...
            ((abs(props_left(j).Orientation)) >= 45)) ))
        div_Ar_left(j) = NaN;
        continue;
    else
        el_fit = fit_ellipse(props_left(j).PixelList(:,1), props_left(j).PixelList(:,2),target_fig);
        el_Area = pi * (el_fit.long_axis/2) * (el_fit.short_axis / 2);
        
        disp(props_left(j).Area);
        disp(el_Area);
        
        div_Ar_left(j) = props_left(j).Area / el_Area;
        if (div_Ar_left(j) < 0.8)
            continue;
        else
            left_x = [left_x props_left(j).Centroid(:,1)];
            left_y = [left_y props_left(j).Centroid(:,2)];            
        end
       
    end
end

props_right = regionprops(BW_right, 'all');
for j = 1:size(props_right, 1)
    if(props_right(j).Area < thresh || ...
            ( ((abs(props_right(j).Orientation) <= 90) && ...
            ((abs(props_right(j).Orientation)) >= 45)) ) )       
        div_Ar_right(j) = NaN;        
        continue;
    else
        el_fit = fit_ellipse(props_right(j).PixelList(:,1), props_right(j).PixelList(:,2));
        el_Area = pi * (el_fit.long_axis/2) * (el_fit.short_axis / 2);
        disp("");
        disp(props_right(j).Area);
        disp(el_Area);
        
        
        div_Ar_right(j) = props_right(j).Area / el_Area;
        if (div_Ar_right(j) < 0.8)
            continue;
        else
            right_x = [right_x props_right(j).Centroid(:,1)];
            right_y = [right_y props_right(j).Centroid(:,2)];
        end
    end
end

if (length(left_x) > 1)
    left_sorted = sort(left_x, 'ascend');
    diff_left = abs(bsxfun(@minus, left_sorted, left_sorted'));
    diff_left = triu(diff_left);
    [~, lc] = find(diff_left > 30);
    if isempty(lc)
        left_x = mean(left_sorted);
    else
        left_x = mean(left_sorted(1:lc-1));
    end
end

if (length(right_x) > 1)
    right_sorted = sort(right_x, 'ascend');
    diff_right = abs(bsxfun(@minus, right_sorted, right_sorted'));
    diff_right = triu(diff_right);
    [~, rc] = find(diff_right > 30);
    if isempty(rc)
        right_x = mean(right_sorted);
    else
        right_x = mean(right_sorted(1:rc-1));
    end
end

if (length(left_y) > 1)
    left_sorted_y = sort(left_y, 'ascend');
    diff_left_y = abs(bsxfun(@minus, left_sorted_y, left_sorted_y'));
    diff_left_y = triu(diff_left_y);
    [~, lc_y] = find(diff_left_y > 30);
    if isempty(lc_y)
        left_y = mean(left_sorted_y);
    else
        left_y = mean(left_sorted_y(1:lc_y-1));
    end
end

if (length(right_y) > 1)
    right_sorted_y = sort(right_y, 'ascend');
    diff_right_y = abs(bsxfun(@minus, right_sorted_y, right_sorted_y'));
    diff_right_y = triu(diff_right_y);
    [~, rc_y] = find(diff_right_y > 30);
    if isempty(rc_y)
        right_y = mean(right_sorted_y);
    else
        right_y = mean(right_sorted_y(1:rc_y-1));
    end
end
