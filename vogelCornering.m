function [A_y, WT, WTF, WTR ,phif, phir ,wfin, wfout ,wrin, wrout ,IA_f_in ,IA_f_out, r, a_f, a_r ,F_fin ,F_fout ,f_x ,rescale ,F_rin ,F_rout ,F_y, M_z_diff, M_z, AY, diff_AY] = vogelCornering(a,b,Cd,IA_gainf,IA_gainr,twf,KPIf,cg,W,twr,LLTD,rg_r,rg_f,casterf,KPIr,deltar,A,sf_y,T_lock,delta,beta,V,R,wf,wr, IA_0f,IA_0r)
            
global r_max accel grip deccel lateral cornering gear shift_points...
    top_speed r_min path_boundaries tire_radius shift_time...
    powertrainpackage track_width path_boundaries_ax

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

end