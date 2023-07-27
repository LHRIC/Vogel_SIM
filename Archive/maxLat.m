function [AY FY SA] = maxLat(Ay,Fz,twf,twr,cg,W,LLTD,A)
        
        global OptimParameterSet

        WT = Ay*cg*W/mean([twf twr]);
        WTF = WT*LLTD;
        WTR = WT*(1-LLTD);
        Fz(1) = Fz(1)+WTF; %Fz1 = Front outer
        Fz(2) = Fz(2)-WTF; %Fz2 = Front inner
        Fz(3) = Fz(3)+WTR; %Fz3 = Rear outer
        Fz(4) = Fz(4)-WTR; %Fz4 = Rear inner
        
        SA_n = 0:.01:.28;
        FY_n = zeros(length(SA_n),1);
        for m = 1:4
            for n = 1:length(SA_n)
                FY_n(n) = -MF52_Fy_fcn([SA_n(n) Fz(m) 0]);
            end
            [FY(m),I]= max(FY_n);
            SA(m) = SA_n(I);
        end

        AY = sum(FY)/W;

end