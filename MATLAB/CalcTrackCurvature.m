clear all;

track = readtable("../trajectory/23_michigan_autox_ft.csv");

r_max = 36;

X = track.X * 0.3048;
Y = track.Y * 0.3048;

path_points = [X, Y];

KT = LineCurvature2D([X, Y]);
KT = KT(~isnan(KT));
% smallvalues = ;
KT(find(abs(KT)<1/r_max)) = 1/r_max;
RT = abs(1./KT);

Tc = array2table(KT,'VariableNames',{'curvature'});
writetable(Tc, '../trajectory/23_michigan_autox_curvature_m.csv');
T = array2table(RT,'VariableNames',{'radii'});
writetable(T, '../trajectory/23_michigan_autox_radii_m.csv');