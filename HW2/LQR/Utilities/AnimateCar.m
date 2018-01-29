function AnimateCar(t,q,v,hFig)
%     figure, plot(t,x)

    if ~iscell(q)
        q = {q};
    end
    
    nCars = length(q);
    
%     v = 1;
    
%     carColor = summer(nCars+1);
    carColor = repmat(linspace(0.7,0,nCars)',1,3);
    
    carWidth = 0.5;
    carLength = 1;
    
    carPatch = [ -carLength/2, -carLength/2,  carLength/2,   carLength/2,  -carLength/2;
                -carWidth/2, carWidth/2, carWidth/2, -carWidth/2, -carWidth/2];
    
%% Extract the data
    
    x = cell(nCars,1);
    y = x;
    theta = x;
    car = x;
    
    xMin = inf;
    xMax = -inf;
    yMin = inf;
    yMax = -inf;
    
            
    for i = 1:nCars
        if size(q{i},2) == 2
            y{i} = q{i}(:,1);
            theta{i} = q{i}(:,2);
            % Integrate to get x position of the car
            x{i} = [0; cumsum(v*cos(theta{i}(1:end-1)).*diff(t))];
        else
            x{i} = q{i}(:,1);
            y{i} = q{i}(:,2);
            theta{i} = q{i}(:,3);
        end
        
        th_ = theta{i}(1);
        rotMat_ = [cos(th_) -sin(th_);
                   sin(th_)  cos(th_)];
        carPatchRot_ = rotMat_*carPatch;
        
        % These are for plotting
        xMin = min(min(x{i}),xMin);
        xMax = max(max(x{i}),xMax);
        yMin = min(min(y{i}),yMin);
        yMax = max(max(y{i}),yMax);
    end
    
%% Set up the plots
    if nargin > 3
        figure(hFig)
    else 
        figure(1)
        clf,hold on
    end
    axis equal
    box off
    set(gca,'xcolor',get(gcf,'color'),'xtick',[],'xticklabel',[]);
    set(gca,'ycolor',get(gcf,'color'),'ytick',[],'yticklabel',[]);
    xlim([xMin - carLength,xMax + carLength])
    ylim([yMin - carLength,yMax + carLength])
    
    for i = 1:nCars
        % Plot the initial car
        car{i} = patch(carPatchRot_(1,:) + x{i}(1), carPatchRot_(2,:) + y{i}(1), carColor(i,:));
        
        % Plot the trajectory
        plot(x{i},y{i},'color',carColor(i,:))
    end
    
%% Animate the cars

    t_curr = 0;
    tic
    
    while t_curr < t(end)
        [~,ind_curr] = min(abs(t_curr - t));
        for i = 1:nCars
            th_ = theta{i}(ind_curr);
            rotMat_ = [cos(th_) -sin(th_);
                       sin(th_)  cos(th_)];
            carPatchRot_ = rotMat_*carPatch;
            set(car{i},'XData',carPatchRot_(1,:) + x{i}(ind_curr),'YData',carPatchRot_(2,:) + y{i}(ind_curr))
        end
%         pause(1/100);
        drawnow
        t_curr = toc;
    end
    
    hold off
end