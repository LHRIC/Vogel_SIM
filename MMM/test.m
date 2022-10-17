clear all
track = readtable('Track Creation/17_lincoln_endurance_track.xls');

path_points = [track.X, track.Y]/1000;

K = LineCurvature2D(path_points);

 r=sort(rand(15,1))*2*pi;
 Vertices=[sin(r) cos(r)]*10;
 Lines=[(1:size(Vertices,1))' (2:size(Vertices,1)+1)']; Lines(end,2)=1;
 k=LineCurvature2D(Vertices,Lines);