function [lap_time, time_elapsed, velocity, acceleration, lateral_accel, gear_counter, path_length, weights, distance] = lap_information(track, showPlots)
global path_boundaries r_min r_max cornering accel grip deccel lateral...
    shift_points top_speed shift_time
%% Generate vehicle trajectory
% this is done the same way as in the main lap sim code so I will not
% replicate that explanation here


% path_positions(end+1) = path_positions(1);
% path_positions(end+1) = path_positions(2);

% t = 1:1:length(path_positions);
% for i = 1:1:length(path_positions)
%     coeff = path_boundaries(i,1:2);
%     x2 = max(path_boundaries(i,3:4));
%     x1 = min(path_boundaries(i,3:4));
%     position = path_positions(i);
%     x3 = x1+position*(x2-x1);
%     y3 = polyval(coeff,x3);
%     path_points(i,:) = [x3 y3];             
% end


disp('INTERVAL IS CHANGED')
interval = 10;
% interval = 1000;
sections = 10;
% sections = 6000;
VMAX = top_speed;
r_min = 4.5;

t = 1:height(track);
path_points = [track.X, track.Y]/1000;

KT = LineCurvature2D(path_points);
KT = KT(~isnan(KT));
% smallvalues = ;
KT(find(abs(KT)<1/r_max)) = 1/r_max;
RT = abs(1./KT);
RT(end-2:end) = [];

figure
patch(path_points(:,1),path_points(:,2),KT,KT,'EdgeColor','interp','FaceColor','none')
h = colorbar;
set(get(h,'title'),'string','Curvature');
set(gca,'XTick',[], 'YTick', [])

x = linspace(1,t(end-1),10000);
%ppv = csaps(t,path_points,1);
%vehicle_path = ppval(ppv,x);
if showPlots == true
figure
plot(track.X/1000,track.Y/1000,'o',track.X(1)/1000,track.Y(1)/1000,'o')
%hold on
%fnplt(ppv)
%hold on
end

% x = linspace(1,t(end-1),1000);
% ppv = interp1([1:length(path_points)],path_points,x,'makima');
% vehicle_path = ppv';

%% Traverse the track
track_points = path_points';
track_points = [track_points(:,length(track_points)-2) track_points(:,1:end-2)];
% [LT,RT,KT] = curvature(track_points');
path_length = arclength(track_points(1,:),track_points(2,:),'pchip');

% for each point along the track, find the maximum theoretical speed
% possible for that specific point, as well as the incremental distance
% travelled
dist = zeros(1,length(RT));
segment = zeros(1,length(RT));
Vmax = zeros(1,length(RT));
y1 = zeros(1,length(RT));
y2 = zeros(1,length(RT));
for i = 1:length(RT)
    segment(i) = i;
    r = max(r_min,RT(i));
    r = min(r,r_max);
    RT(i) = r;
    r = RT(i); % radius of curvature of that segment
    if (r < 8)
    r = 8;
    end
    Vmax(i) = min(VMAX,polyval(cornering,r));
    x1(i) = track_points(1,i+1);
    x2(i) = track_points(1,i+2);
    y1(i) = track_points(2,i+1);
    y2(i) = track_points(2,i+2);
    dist(i) = sqrt((x1(i)-x2(i))^2+(y2(i)-y1(i))^2);
end
%% Initiate forward sim
count = 0;
    v = 20*.3048;
    vel = v;
    gears = find((shift_points-vel)>0);
    gear = gears(1)-1;
    newgear = gear;
    time_shifting = 0;
for i = 1:1:length(segment) % for each track segment
    d = dist(i); % distance travelled over segment
    r = RT(i); % radius of curvature of that segment
    %gear = newgear;
    % find what gear you are in
    gears = find((shift_points-vel)>0);
    newgear = gears(1)-1;
    
    % if you are upshifting, turn on the upshift variable (referenced
    % later)
    if newgear > gear
        shifting = 1;
    else
        shifting = 0;
    end
    % find maximum speed possible through segment
    vmax = min(VMAX,polyval(cornering,r));
    if vmax < 0
        vmax = VMAX;
    end
    % find acceleration capabilities for your current speed
    AX = fnval(accel,vel);
    AY = polyval(lateral,vel);
    dd = d/interval;
    % now, for each little interval within the larger segment:
    for j = 1:1:interval
        count = count+1;
        % log gear selection
        vehicle_gear(count) = gear;
        % current lateral acceleration
        ay_f(count) = vel^2/(r*9.81);
        % shifting logic code:
        if shifting == 1 & vel < vmax;
            % if you are shifting, don't accelerate
            dt_f(count) = dd/vel;
            % keep track of how much time has been spent shifting
            time_shifting = time_shifting+dt_f(count);
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
            vel = vel;
        elseif vel < vmax
            % if you are not shifting, and are going below max possible
            % speed,
            % find potential acceleration available:
            ax_f(count) = AX*(1-(min(AY,ay_f(count))/AY)^2); %%%?????????????????shouldn't this be like Ax_max = 
            %%???? shouldn't this be like Ax = Ax_max*sqrt(1-(ay/Ay_max)^2)
            tt = roots([0.5*9.81*ax_f(count) vel -dd]);
            % accelerate according to that capacity, update speed and
            % position accordingly
            dt_f(count) = max(tt);
            dv = 9.81*ax_f(count)*dt_f(count);
            dvmax = vmax-vel;
            dv_f(count) = min(dv,dvmax);
            v_f(count) = vel+dv_f(count); 
            vel = v_f(count);
            gears = find((shift_points-vel)>0);
            newgear = gears(1)-1;
            if newgear > gear
                shifting = 1;
            end
        else
            % otherwise you must be maxed out already, so no more
            % acceleration
            vel = vmax;
            dt_f(count) = dd/vel;
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
        end
        % once you have been shifting long enough that the entire
        % pre-specified shift time has elapsed, you can turn the shifting
        % variable back off:
        if time_shifting > shift_time
            shifting = 0;
            time_shifting = 0;
            gear = newgear;
        end
    end


    if shifting == 1
        gear = gear;
    else
        gear = newgear;
    end
    disp(gear);
end

dtot = 0;    
for i = 1:1:count
    j = ceil(i/interval);
    dd = dist(j)/interval;
    dtot = dtot+dd;
    distance(i) = dtot;
end
%% Re run, with new starting velocity
V0 = v_f(end);
% initiate reverse sim, it's the same premise but going backwards, and you
% "accelerate" backwards as you brake
count = length(segment)*interval+1;
    v = V0;
    vel = v;
for i = length(segment):-1:1
    d = dist(i);

    r = RT(i);


    vmax = min(VMAX,polyval(cornering,r));
    if vmax < 0
        vmax = VMAX;
    end
    AX = polyval(deccel,vel);
    AY = polyval(lateral,vel);
    dd = d/interval;
    for j = 1:1:interval
        count = count-1;
        ay_r(count) = vel^2/(r*9.81);
        if vel < vmax
            ax_r(count) = AX*(1-(min(AY,ay_r(count))/AY)^2);
            tt = roots([0.5*9.81*ax_r(count) vel -dd]);
            dt_r(count) = max(tt);
            dv = 9.81*ax_r(count)*dt_r(count);
            dvmax = vmax-vel;
            dv_r(count) = min(dv,dvmax);
            v_r(count) = vel+dv_r(count); 
            vel = v_r(count);
        else
            vel = vmax;
            dt_r(count) = dd/vel;
            ax_r(count) = 0;
            v_r(count) = vel;
            dv_r(count) = 0;
        end
    end

    
end

% Initiate forward sim again, knowing your starting velocity now

count = 0;
v = V0;
vel = v;
gears = find((shift_points-vel)>0);
gear = gears(1)-1;
newgear = gear;
time_shifting = 0;
for i = 1:1:length(segment)
    d = dist(i);

    r = RT(i);
    %gear = newgear;
    gears = find((shift_points-vel)>0);
    newgear = gears(1)-1;
    
    if newgear > gear
        shifting = 1;
    else
        shifting = 0;
    end

    vmax = min(VMAX,polyval(cornering,r));
    if vmax < 0
        vmax = VMAX;
    end
    AX = fnval(accel,vel);
    AY = polyval(lateral,vel);
    dd = d/interval;
    for j = 1:1:interval
        count = count+1;
        vehicle_gear(count) = gear;
        ay_f(count) = vel^2/(r*9.81);
        if shifting == 1 & vel < vmax;
            dt_f(count) = dd/vel;
            time_shifting = time_shifting+dt_f(count);
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
            vel = vel;
        elseif vel < vmax
            ax_f(count) = AX*(1-(min(AY,ay_f(count))/AY)^2);
            tt = roots([0.5*9.81*ax_f(count) vel -dd]);
            dt_f(count) = max(tt);
            dv = 9.81*ax_f(count)*dt_f(count);
            dvmax = vmax-vel;
            dv_f(count) = min(dv,dvmax);
            v_f(count) = vel+dv_f(count); 
            vel = v_f(count);
            gears = find((shift_points-vel)>0);
            newgear = gears(1)-1;
            if newgear > gear
                shifting = 1;
            end
        else
            vel = vmax;
            dt_f(count) = dd/vel;
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
        end
        if time_shifting > shift_time
            shifting = 0;
            time_shifting = 0;
            gear = newgear;
        end
    end
    if shifting == 1
        gear = gear;
    else
        gear = newgear;
    end
end

%% combine results
VD = v_f-v_r;
forw = find(VD>=0);
back = find(VD<0);
velocity = zeros(1,length(VD));
t_elapsed = 0;
classifier = [];
for i = 1:1:length(VD)
    if VD(i)<0
        velocity(i) = v_f(i);
        dtime(i) = dt_f(i);
        acceleration(i) = ax_f(i);
        lateral_accel(i) = ay_f(i);
        %shift_timer(i) = shift_timer(i);
    else
        velocity(i) = v_r(i);
        dtime(i) = dt_r(i);
        acceleration(i) = -ax_r(i);
        lateral_accel(i) = ay_r(i);
        %shift_timer(i) = 0;
    end
    t_elapsed = t_elapsed+dtime(i);
    time_elapsed(i) = t_elapsed;
end
AY_outlier = find(lateral_accel > polyval(lateral,top_speed+1));
lateral_accel(AY_outlier) = polyval(lateral,top_speed+1);
throttle = 0;
brake = 0;
corner = 0;
for i = 1:1:length(VD)
    if acceleration(i)>0
        throttle = throttle+acceleration(i)*dtime(i);
    elseif acceleration(i) < 0
        brake = brake-acceleration(i)*dtime(i);
    end
    corner = corner + lateral_accel(i)*dtime(i);
end
summ = throttle+brake+corner;
weights = [throttle/summ brake/summ corner/summ];
tloc = find(acceleration>.25);
t_t = sum(dtime(tloc));
bloc = find(acceleration<-.25);
t_b = sum(dtime(bloc));
cloc = find(lateral_accel>.25);
t_c = sum(dtime(cloc));
summ = t_t+t_b+t_c;
weights = [t_t/summ t_b/summ t_c/summ];
%figure
%plot(distance,velocity)

%% Plot Results
%figure
for i = 1:1:length(track_points)-2
    V_plot(i) = mean(velocity(i*interval-interval+1:i*interval));
end
if showPlots == true
figure
scatter(track_points(1,2:end-1),track_points(2,2:end-1),100,V_plot,'marker','.')
title('2019 Michigan Endurance Simulation Track Summary')
h = colorbar;
set(get(h,'title'),'string','Velocity (V) [m/s]');
set(gca,'XTick',[], 'YTick', [])
grid on 
grid minor

figure
patch(track_points(1,2:end-1),track_points(2,2:end-1),V_plot,V_plot,'EdgeColor','interp','FaceColor','none')
colormap('cool')
h = colorbar;
set(get(h,'title'),'string','Velocity (V) [m/s]');
set(gca,'XTick',[], 'YTick', [])
end

%% Gear Counter
for i = 1:1:length(velocity)
V = velocity(i);
    gears = find((shift_points-V)>0);
    gear = gears(1)-1;
gear_counter(i) = gear;
end
lap_time = t_elapsed;

for i = 1:1:length(lateral_accel)
    index = floor((i-1)/interval)+1;
    axis(i) = sign(KT(index));
end
lateral_accel = lateral_accel.*axis;