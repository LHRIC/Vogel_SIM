function [combined_curve] = CurveIntersect(curve1, curve2)

%% Debugging Code
%     clc;
%     close all;
%     clear all;
% 
%     curve2 = [linspace(5,10,100).',(linspace(0,10,100).').^2];
%     curve1 = [linspace(0,5,100).', (linspace(0,10,100).').*2-10];
%     
%     figure(); hold on;
%         plot(curve1(:,1), curve1(:,2))
%         plot(curve2(:,1), curve2(:,2))

%% Main Function Code
    % placeholder variables 
    x1 = curve1(:,1); y1 = curve1(:,2);
    x2 = curve2(:,1); y2 = curve2(:,2);

    % Re-fits curve domains to unit step size
    xq1 = ceil(x1(1)):0.1:floor(x1(end)); % new x domain with unit step
    xq2 = ceil(x2(1)):0.1:floor(x2(end)); % new x domain with unit step
    p1 = pchip(x1, y1, xq1); % interpolates y data over new domain
    p2 = pchip(x2, y2, xq2); % interpolates y data over i over new domain
    x1 = xq1.'; x2 = xq2.'; y1 = p1.'; y2 = p2.'; % old data replacement
    [int, int1_D, int2_D] = intersect(x1,x2);

    if isempty(int)
        error("Curves do not have overlapping domains!") 
    else 
        clear int
        % calculate curve difference
        diff = y1(int1_D(1):int1_D(end)) - y2(int2_D(1):int2_D(end));
        [~, int_I] = min(abs(diff));
        [int_val] = diff(int_I);

        if abs(int_val) < mean([range(y1),range(y2)])/1000
%             disp("Curve Intersection!")
                dx1 = int_I+int1_D(1)-1;
                dx2 = int_I+int2_D(1);
                x = [x1(1:dx1); x2(dx2:end)];
                y = [y1(1:dx1); y2(dx2:end)];
        else
%             disp("No Curve Intersection!")
            if x1(1) < x2(1)
%                 disp("Curve 1 on top")
                dx2 = find(x2==x1(end))+1;
                x = [x1; x2(dx2:end)];
                y = [y1; y2(dx2:end)];
            else
%                 disp("Curve 2 on top")                
                dx1 = find(x1==x2(end))+1;
                x = [x2; x1(dx1:end)];
                y = [y2; y1(dx1:end)];
            end 
        end 
    end 

    combined_curve = [x,y];
end 