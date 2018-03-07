figure(1)
ShowLaserScan(scan(610))
%%
dummy_scan = scan(610);
%%
coords = LaserScanToCartesian(scan(610));
x = coords(:,1);
y = -coords(:,2);

refs = dummy_scan.reflectances;
% mean_ref = mean(refs);
mean_ref = 640;
ref_mask = refs>mean_ref;
x = x(ref_mask);
y = y(ref_mask);
filtered_refs = refs(ref_mask);

Y = pdist([x,y],'cityblock');
Z = linkage(Y,'average');
T = cluster(Z,'cutoff',0.10, 'criterion', 'distance');
%%
figure(2)
scatter(x, y, 10, T , 'filled')
hold on;

answer = [];
residual = [];
cluster_mean_ref = [];

for i = 1:1:max(T)
    mask = T == i;
    
    if sum(mask)<5
        continue;
    end
    
    x_test = x(mask);
    y_test = y(mask);
    edge_point = mean([x_test,y_test]);
    new_centre = (0.057*edge_point)/norm(edge_point) + edge_point;
    res = sum(abs(0.06^2 - ((x_test - new_centre(1)).^2 + (y_test - new_centre(2)).^2)));
    avg_res = abs(res)/sum(mask);
    
    if and(new_centre(1)>0, avg_res<0.001)
        hold on;
        circle(new_centre(1),new_centre(2),0.06)
        answer = [answer, i];
        residual = [residual,avg_res];
        cluster_mean_ref = [cluster_mean_ref, mean(filtered_refs(mask))];
    end
     
end
