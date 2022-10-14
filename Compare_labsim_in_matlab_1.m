%{
1) import multiple .mat files at once to compare them (they need to struct and
with the same variable names)
2) choose the number of nested graphs you want to see at once
3) individualy choose the x then the y variable for each graph
%}
%{
Initializes some variables. Imports all data into 'storedData'. User input
number of graps saved into 'numberOfGraphs'. User input variables into 2D
array 'XYVar'. If the variable only has a few points it must be graphed as
a scatter instead of a plot so 'XYVarScatterOrPlot' would be flipped to 1.
Graphs the scatters and plots in tiles and with labels and a legend.
%}

clear all, close all, clc

%Variables
numberOfGraphs = 1;
subName = 'LapSimOutput';
XYVar = ["plot1x","plot2x","plot3x","plot4x","plot5x","plot6x","plot7x","plot8x";
       "plot1y","plot2y","plot3y","plot4y","plot5y","plot6y","plot7y","plot8y"]
XYVarScatterOrPlot = [0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0]          %value set to 1 if it is to be scattered instead of graphed


%Import file structs
[file, path, indx] = uigetfile('*.mat','MultiSelect','on');
for i=1:length(file)
    storedData{1,[i]} = load(file{1,[i]});      %storedData has all imported data
end

%Prompt user to request data
list = {'1','2','3','4','5','6','7','8'};
[xValue,alwaysOne] = listdlg('PromptString',{'How many graphs you want???'},'SelectionMode','single','ListString',list);
numberOfGraphs = xValue;


for d=1:numberOfGraphs

    list = {'Time Elapsed','Distance','Velocity','Acceleration','Lateral Acceleration','Gear Counter','Time Elapsed AX','Distance AX','Velocity AX','Acceleration AX','Lateral Acceleration AX','Gear Counter AX','Path Length','Path Length AX','Weights AX','Accel Time','Endurance Scoore','Autocross Score','Accel Score','Skidpad Score','Weights','Lap Time','Lap Time AX'};
    [xValue,alwaysOne] = listdlg('PromptString',{'Select an X variable', 'GRAPH' d},'SelectionMode','single','ListString',list);
        if xValue == 1
            XYVar(1,d) = 'time_elapsed'
        elseif xValue == 2
            XYVar(1,d) = 'distance'
        elseif xValue == 3
            XYVar(1,d) = 'velocity'
        elseif xValue == 4
            XYVar(1,d) = 'acceleration'
        elseif xValue == 5
            XYVar(1,d) = 'lateral_accel'
        elseif xValue == 6
            XYVar(1,d) = 'gear_counter'
        elseif xValue == 7
            XYVar(1,d) = 'time_elasped_ax'
        elseif xValue == 8
            XYVar(1,d) = 'distance_ax'
        elseif xValue == 9
            XYVar(1,d) = 'velocity_ax'
        elseif xValue == 10
            XYVar(1,d) = 'acceleration_ax'
        elseif xValue == 11
            XYVar(1,d) = 'lateral_accel_ax'
        elseif xValue == 12
            XYVar(1,d) = 'gear_counter_ax'
        elseif xValue == 13
            XYVar(1,d) = 'path_length'
            XYVarScatterOrPlot(d,1) = 1;
        elseif xValue == 14
            XYVar(1,d) = 'path_length_ax'
            XYVarScatterOrPlot(d,1) = 1;
        elseif xValue == 15
            XYVar(1,d) = 'weights_ax'
            XYVarScatterOrPlot(d,1) = 1;
        elseif xValue == 16
            XYVar(1,d) = 'accel_time'
            XYVarScatterOrPlot(d,1) = 1;
        elseif xValue == 17
            XYVar(1,d) = 'Endurance_Score'
            XYVarScatterOrPlot(d,1) = 1;
        elseif xValue == 18
            XYVar(1,d) = 'Autocross_Score'
            XYVarScatterOrPlot(d,1) = 1;
        elseif xValue == 19
            XYVar(1,d) = 'Accel_Score'
            XYVarScatterOrPlot(d,1) = 1;
        elseif xValue == 20
            XYVar(1,d) = 'Skidpad_Score'
            XYVarScatterOrPlot(d,1) = 1;
        elseif xValue == 21
            XYVar(1,d) = 'weights'
            XYVarScatterOrPlot(d,1) = 1;
        elseif xValue == 22
            XYVar(1,d) = 'laptime'
            XYVarScatterOrPlot(d,1) = 1;
        else xValue == 23
            XYVar(1,d) = 'laptime_ax'
            XYVarScatterOrPlot(d,1) = 1;
        end
    
    list = {'Time Elapsed','Distance','Velocity','Acceleration','Lateral Acceleration','Gear Counter','Time Elapsed AX','Distance AX','Velocity AX','Acceleration AX','Lateral Acceleration AX','Gear Counter AX','Path Length','Path Length AX','Weights AX','Accel Time','Endurance Scoore','Autocross Score','Accel Score','Skidpad Score','Weights','Lap Time','Lap Time AX'};
    [yValue,alwaysOne] = listdlg('PromptString',{'Select a Y variable', 'Graph' d},'SelectionMode','single','ListString',list);
        if yValue == 1
            XYVar(2,d) = 'time_elapsed'
        elseif yValue == 2
            XYVar(2,d) = 'distance'
        elseif yValue == 3
            XYVar(2,d) = 'velocity'
        elseif yValue == 4
            XYVar(2,d) = 'acceleration'
        elseif yValue == 5
            XYVar(2,d) = 'lateral_accel'
        elseif yValue == 6
            XYVar(2,d) = 'gear_counter'
        elseif yValue == 7
            XYVar(2,d) = 'time_elasped_ax'
        elseif yValue == 8
            XYVar(2,d) = 'distance_ax'
        elseif yValue == 9
            XYVar(2,d) = 'velocity_ax'
        elseif yValue == 10
            XYVar(2,d) = 'acceleration_ax'
        elseif yValue == 11
            XYVar(2,d) = 'lateral_accel_ax'
        elseif yValue == 12
            XYVar(2,d) = 'gear_counter_ax'
        elseif yValue == 13
            XYVar(2,d) = 'path_length'
            XYVarScatterOrPlot(d,2) = 1;
        elseif yValue == 14
            XYVar(2,d) = 'path_length_ax'
            XYVarScatterOrPlot(d,2) = 1;
        elseif yValue == 15
            XYVar(2,d) = 'weights_ax'
            XYVarScatterOrPlot(d,2) = 1;
        elseif yValue == 16
            XYVar(2,d) = 'accel_time'
            XYVarScatterOrPlot(d,2) = 1;
        elseif yValue == 17
            XYVar(2,d) = 'Endurance_Score'
            XYVarScatterOrPlot(d,2) = 1;
        elseif yValue == 18
            XYVar(2,d) = 'Autocross_Score'
            XYVarScatterOrPlot(d,2) = 1;
        elseif yValue == 19
            XYVar(2,d) = 'Accel_Score'
            XYVarScatterOrPlot(d,2) = 1;
        elseif yValue == 20
            XYVar(2,d) = 'Skidpad_Score'
            XYVarScatterOrPlot(d,2) = 1;
        elseif yValue == 21
            XYVar(2,d) = 'weights'
            XYVarScatterOrPlot(d,2) = 1;
        elseif yValue == 22
            XYVar(2,d) = 'laptime'
            XYVarScatterOrPlot(d,2) = 1;
        else yValue == 23
            XYVar(2,d) = 'laptime_ax'
            XYVarScatterOrPlot(d,2) = 1;
        end

end


if XYVarScatterOrPlot(d,1) == 1 && XYVarScatterOrPlot(d,2) == 0
    warndlg('You are making a SCATTER plot with like a million data points, this is because one of your variables has a bunch of values and the other does not, it might take a while... or not work at all... rethink your life decisions')
elseif XYVarScatterOrPlot(d,1) == 0 && XYVarScatterOrPlot(d,2) == 1
    warndlg('You are making a SCATTER plot with like a million data points, this is because one of your variables has a bunch of values and the other does not, it might take a while... or not work at all... rethink your life decisions')
end


%Plot structs
for d=1:numberOfGraphs

    nexttile

    if XYVarScatterOrPlot(d,1) == 0 && XYVarScatterOrPlot(d,2) == 0
        fprintf('going to PLOT')
        plot(storedData{1, 1}.(subName).(XYVar(1,d)), storedData{1, 1}.(subName).(XYVar(2,d)),'DisplayName',file{1, 1})
        hold on
        for a=2:length(file)
            plot(storedData{1, [a]}.(subName).(XYVar(1,d)), storedData{1, [a]}.(subName).(XYVar(2,d)),'DisplayName',file{1, [a]})
        end
        hold off
    else
        scatter(storedData{1, 1}.(subName).(XYVar(1,d)), storedData{1, 1}.(subName).(XYVar(2,d)),'DisplayName',file{1, 1})
        hold on
        for a=2:length(file)
            scatter(storedData{1, [a]}.(subName).(XYVar(1,d)), storedData{1, [a]}.(subName).(XYVar(2,d)),'DisplayName',file{1, [a]})
        end
        hold off
    end


    xlabel((XYVar(1,d)))
    ylabel((XYVar(2,d)))
    legend

end