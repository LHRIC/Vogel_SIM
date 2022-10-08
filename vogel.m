function [output] =vogel(x,a,b,Cd,IA_gainf,IA_gainr,twf,KPIf,cg,W,twr,LLTD,rg_r,rg_f,casterf,KPIr,deltar,sf_y,T_lock,R,wf,wr,WTR,IA_0f,IA_0r,A)

global grip
<<<<<<< Updated upstream
        delta = x(1)
        beta = x(2) 
        AYP = x(3)
=======
>>>>>>> Stashed changes
        delta = x(1);
        beta = x(2) ;
        AYP = x(3) ;
        V = sqrt(R*32.2*AYP);
        A_y = V^2/R;
        WT = A_y*cg*W/mean([twf twr])/32.2/12;
        WTF = WT*LLTD;
        WTR = WT*(1-LLTD);
        phif = A_y*rg_f*pi/180/32.2;
        phir = A_y*rg_r*pi/180/32.2;
        wfin = wf-WTF;
        wfout = wf+WTF;
        wrin = wr-WTR;
        wrout = wr+WTR;
        IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
        IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
        IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
        IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
        r = A_y/V;
        a_f = beta+a*r/V-delta;
        a_r = beta-b*r/V;
        F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
        F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
        F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
        rscale = 1-(F_x/W/fnval(grip,V))^2;
        F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
        F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
        F_y = F_fin+F_fout+F_rin+F_rout;
        M_z_diff = F_x*T_lock*twr/2;       
        M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
        AY = F_y/(W/32.2);
        diff_AY = A_y-AY; 
        %minimizing values
        M_z = M_z;
        slipAngle = a_f-deg2rad(-12);
        diff_AY =A_y-AY;
        output = [M_z slipAngle diff_AY];
end