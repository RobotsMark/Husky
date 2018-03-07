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


% Y = pdist(coords,'cityblock');
Y = pdist([x,y],'cityblock');
Z = linkage(Y,'average');
T = cluster(Z,'cutoff',0.10, 'criterion', 'distance');
%%
figure(2)
scatter(x, y, 10, T , 'filled')
hold on;
% trueinds = and(and(coords(:,1)<0.8,coords(:,1)>0.6),and(coords(:,2)<0,coords(:,2)>-0.3));
% trueinds = and(and(coords(:,1)<0.29,coords(:,1)>0.22),and(coords(:,2)<0.1,coords(:,2)>-0.1));
trueinds = and(and(x<0.29,x>0.22),and(y<0.1,y>-0.1));
% 
% xt = x(trueinds);
% yt = y(trueinds);
% 
% Par = CircleFitByPratt([xt,yt]);
% [altxy, altr, residual] = fitcircle([xt,yt]);

% figure()
% scatter(x(trueinds), y(trueinds),'b');
% hold on;
% scatter(Par(1),Par(2),'r')
% circle(Par(1),Par(2),Par(3))

answer = [];
residual = [];

for i = 1:1:max(T)
    mask = T == i;
    
    if sum(mask)<5
        continue;
    end
    
    x_test = x(mask);
    y_test = y(mask);
    [altxy, altr, res] = givenr_fitcircle([x_test,y_test],0.06);
    
    edge = mean([x_test,y_test]);
    new_centre = (0.057*edge)/norm(edge) + edge;
%     scatter (new_centre(1),new_centre(2),'r')
    hold on;
    circle(new_centre(1),new_centre(2),0.06)
    
    res = sum(0.06^2 - ((x_test - new_centre(1)).^2 + (y_test - new_centre(2)).^2));
    
%     figure()
%     scatter(x_test,y_test)
%      circle(altxy(1),altxy(2),altr)
        
%     if and(altr<0.1,altr>0.04)
%         answer = [answer,i];
     residual = [residual,res];
% 
%         circle(altxy(1),altxy(2),altr)
%     end

    % circle(altxy(1),altxy(2),altr)
end


%% New method
