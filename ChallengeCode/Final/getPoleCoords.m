function [ newFeatures ] = getPoleCoords( scan )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

coords = LaserScanToCartesian(scan);
x = coords(:,1);
y = -coords(:,2);

refs = scan.reflectances;
mean_ref = mean(refs);
% mean_ref = 640;
% ref_threshold = max([800,0.65*max(refs)]);
ref_threshold = max([750,0.65*max(refs)]);
ref_mask = refs>ref_threshold;
x = x(ref_mask);
y = y(ref_mask);
filtered_refs = refs(ref_mask);

Y = pdist([x,y],'cityblock');
if ~isempty(Y)
    Z = linkage(Y,'average');
    T = cluster(Z,'cutoff',0.10, 'criterion', 'distance');
    % 
    % figure(2)
    % scatter(x, y, 10, T , 'filled')
    % hold on;

    clusterNum = [];
    residual = [];
    cluster_mean_ref = [];
    outputCoords = [];
    newFeatures = [];

    for i = 1:1:max(T)
        mask = T == i;

        if sum(mask)<3
            continue;
        end

        x_test = x(mask);
        y_test = y(mask);
        test_reflectances = filtered_refs(mask);
        
        edge_point = mean([x_test,y_test]);
        new_centre = (0.057*edge_point)/norm(edge_point) + edge_point;
        res = sum(abs(0.06^2 - ((x_test - new_centre(1)).^2 + (y_test - new_centre(2)).^2)));
        avg_res = abs(res)/sum(mask);

        if and(and(new_centre(1)>0.1, avg_res<0.01),mean(test_reflectances)>800)
%             Convert to centre of robot frame
            feature_x = new_centre(1) + 0.99/2;
            feature_y = new_centre(2);
    %         hold on;
    %         circle(new_centre(1),new_centre(2),0.06)

    % %       Transform into frame with centre of robot  

    %         clusterNum = [clusterNum; i];
    %         residual = [residual;avg_res];
    %         cluster_mean_ref = [cluster_mean_ref; mean(filtered_refs(mask))];
            range = sqrt(feature_x.^2 + feature_y.^2);
            bearing = atan2(feature_y,feature_x);
%             bearing = atan(new_centre);
            
            outputCoords = [range, bearing ];
    %         outputCoords = [outputCoords; new_centre ];
            newFeatures = [newFeatures;outputCoords];
        end
    end
    
    newFeatures = newFeatures';   
else
    newFeatures =[];
end
    

end


    



