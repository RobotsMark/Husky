function [ newFeatures ] = getPoleCoords( scan )
% Uses thresholding and clustering to determine the range and bearing to 
% centres of obstacles/poles in laser scan data from the robot
%   input:  scan - laser scan from robot
%   output: newFeatures - this is an array of [range, bearing] for 
%           each pole found in the scan

% Mark Finean
% March 2018

% Get x,y coords in laser frame
coords = LaserScanToCartesian(scan);
x = coords(:,1);
y = coords(:,2);

% Thresholding on reflectance
refs = scan.reflectances;
ref_threshold = max([750,0.65*max(refs)]);
ref_mask = refs>ref_threshold;
x = x(ref_mask);
y = y(ref_mask);
filtered_refs = refs(ref_mask);

% Clustering
Y = pdist([x,y],'cityblock');
if ~isempty(Y)
    Z = linkage(Y,'average');
    T = cluster(Z,'cutoff',0.10, 'criterion', 'distance'); % Find 10cm diameter clusters
    
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
            range = sqrt(feature_x.^2 + feature_y.^2);
            bearing = atan2(feature_y,feature_x);
            newFeatures = [newFeatures; range, bearing];
        end
    end
    
    newFeatures = newFeatures';   
else
    newFeatures =[];
end
    

end


    



