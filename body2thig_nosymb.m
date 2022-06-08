more off

R_general = @(ux,uy,uz,a)[cos(a)+ux*ux*(1-cos(a)),   ux*uy*(1-cos(a))-uz*sin(a),ux*uz*(1-cos(a))+uy*sin(a);
             uy*ux*(1-cos(a))+uz*sin(a),cos(a)+uy*uy*(1-cos(a)),   uy*uz*(1-cos(a))-ux*sin(a);
             uz*ux*(1-cos(a))-uy*sin(a),uz*uy*(1-cos(a))+ux*sin(a),cos(a)+uz*uz*(1-cos(a))];

    

R_roll_j1  = @(j1)R_general(1,0,0,j1);
R_pitch_j2 = @(j2)R_general(0,1,0,j2);
R_yaw_j3   = @(j3)R_general(0,0,1,j3);

#rpy = [4.18841901e-01 5.23588645e-01 5.38159469e-07] # 0 30 24
rpy = [0.62858704 0.49691578 0.28833546] # -15 30 24

R_rpy = R_yaw_j3(rpy(3)) * R_pitch_j2(rpy(2)) * R_roll_j1(rpy(1)) 

f_rpy_z = R_rpy * [0;0;1]
f_rpy_y = R_rpy * [0;1;0]

    
R_llj1_j1 = @(j1)R_general(-sqrt(2)/2, 0, -sqrt(2)/2,j1);

R_llj2_j2 = @(j2)R_general(0,1,0,j2);

R_llj3_j3 = @(j3)R_general(1,0,0,j3);

R_lHip_j1_j2_j3 = @(j1,j2,j3)R_llj1_j1(j1) * R_llj2_j2(j2) * R_llj3_j3(j3);

fz_j1_j2_j3 = @(j1,j2,j3)R_lHip_j1_j2_j3(j1,j2,j3) * [0;0;1];
fy_j1_j2_j3 = @(j1,j2,j3)R_lHip_j1_j2_j3(j1,j2,j3) * [0;1;0];


q = rpy';                    

fz = fz_j1_j2_j3(q(1),q(2),q(3));
fy = fy_j1_j2_j3(q(1),q(2),q(3));

#fz_goal = [0.3;-0.1;sqrt(1-0.3*0.3-0.1*0.1)]
fz_goal = f_rpy_z
fy_goal = f_rpy_y
err = (fz-fz_goal)'*(fz-fz_goal)

fact=1
prev_err=10
fails=0
while (fz-fz_goal)'*(fz-fz_goal)+(fy-fy_goal)'*(fy-fy_goal) > 1e-5
    step = [rand()-0.5 rand()-0.5 rand()-0.5]'*fact;
    q2 = double(q + step);    
    q2 = mod(q2+pi, 2*pi) - pi;
    fz2 = fz_j1_j2_j3(q2(1),q2(2),q2(3));
    fy2 = fy_j1_j2_j3(q2(1),q2(2),q2(3));
    err = (fz2-fz_goal)'*(fz2-fz_goal)+(fy2-fy_goal)'*(fy2-fy_goal)
    if err < prev_err
        q=q2;
        q*180/pi
        fz=fz2;
        fy=fy2;
        prev_err=err;
        
        q2 = double(q2 + step);    
        fz2 = fz_j1_j2_j3(q2(1),q2(2),q2(3));
        fy2 = fy_j1_j2_j3(q2(1),q2(2),q2(3));
        err = (fz2-fz_goal)'*(fz2-fz_goal)+(fy2-fy_goal)'*(fy2-fy_goal)
        while err < prev_err
           fails=0;
           q=q2;
           q*180/pi
           fz=fz2;
           fy=fy2;
           prev_err=err
        
           q2 = double(q2 + step);    
           fz2 = fz_j1_j2_j3(q2(1),q2(2),q2(3));
           fy2 = fy_j1_j2_j3(q2(1),q2(2),q2(3));
           err = (fz2-fz_goal)'*(fz2-fz_goal)+(fy2-fy_goal)'*(fy2-fy_goal);
           err2=err
        end
    else
        fails=fails+1;
        if fails>2
            fact=fact*0.5
            fails=0;
        end
    end
end    
        
q_goal = q        
 


