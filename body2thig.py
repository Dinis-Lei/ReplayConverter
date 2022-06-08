import numpy as np
import random as rand
import math


np.random.seed(3)

print(np.random.random())
print(np.random.random())
print(np.random.random())



"""
    R_general = [cos(a)+ux*ux*(1-cos(a)),   ux*uy*(1-cos(a))-uz*sin(a),ux*uz*(1-cos(a))+uy*sin(a);
             uy*ux*(1-cos(a))+uz*sin(a),cos(a)+uy*uy*(1-cos(a)),   uy*uz*(1-cos(a))-ux*sin(a);
             uz*ux*(1-cos(a))-uy*sin(a),uz*uy*(1-cos(a))+ux*sin(a),cos(a)+uz*uz*(1-cos(a))];
"""

def R_general(ux, uy, uz, a):
    return np.array([
        np.cos(a) + ux*ux*(1-np.cos(a)), ux*uy*(1-np.cos(a))-uz*np.sin(a), ux*uz*(1-np.cos(a)) + uy*np.sin(a),
        uy*ux*(1-np.cos(a))+uz*np.sin(a),np.cos(a)+uy*uy*(1-np.cos(a)),   uy*uz*(1-np.cos(a))-ux*np.sin(a),
        uz*ux*(1-np.cos(a))-uy*np.sin(a),uz*uy*(1-np.cos(a))+ux*np.sin(a),np.cos(a)+uz*uz*(1-np.cos(a))
    ]).reshape(3,3)

def rot_z(val):
    return np.array([np.cos(val), -np.sin(val), 0, 0, np.sin(val), np.cos(val), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]).reshape(4,4)

# print(R_general(0,0,1,np.pi/4))
# print(rot_z(np.pi/4))

R_roll_j1  = lambda j1: R_general(1,0,0,j1)
R_pitch_j2 = lambda j2: R_general(0,1,0,j2)
R_yaw_j3   = lambda j3: R_general(0,0,1,j3)

rpy = np.array([0.62858704, 0.49691578, 0.28833546]) # -15 30 24
print(f"{rpy = }")

R_rpy = R_yaw_j3(rpy[2]) @ R_pitch_j2(rpy[1]) @ R_roll_j1(rpy[0]) 
print(f"{R_rpy = }")


f_rpy_z = R_rpy @ np.array([0, 0, 1]).reshape(3,1)
f_rpy_y = R_rpy @ np.array([0, 1, 0]).reshape(3,1)


print(f"{f_rpy_z = }")
print(f"{f_rpy_y = }")


R_llj1_j1 = lambda j1: R_general(-np.sqrt(2)/2, 0, -np.sqrt(2)/2,j1)
R_llj2_j2 = lambda j2: R_general(0,1,0,j2)
R_llj3_j3 = lambda j3: R_general(1,0,0,j3)

R_lHip_j1_j2_j3 = lambda j1,j2,j3: R_llj1_j1(j1) @ R_llj2_j2(j2) @ R_llj3_j3(j3)

fz_j1_j2_j3 = lambda j1,j2,j3: R_lHip_j1_j2_j3(j1,j2,j3) @ np.array([0, 0, 1]).reshape(3,1)
fy_j1_j2_j3 = lambda j1,j2,j3: R_lHip_j1_j2_j3(j1,j2,j3) @ np.array([0, 1, 0]).reshape(3,1)

q = rpy.reshape(3,1)

fz = fz_j1_j2_j3(q[0],q[1],q[2])
fy = fy_j1_j2_j3(q[0],q[1],q[2])
fz_goal = f_rpy_z
fy_goal = f_rpy_y
print(f"{fz_goal = }")
print(f"{fy_goal = }")
err = (fz-fz_goal).T@(fz-fz_goal)

print(f"{err = }")

fact=1.0
prev_err=10
fails=0

c = 0
while (fz-fz_goal).T@(fz-fz_goal)+(fy-fy_goal).T@(fy-fy_goal) > 1e-5:

    # if( c > 3):
    #     break
    c += 1
    step = np.array([np.random.random()-0.5, np.random.random()-0.5, np.random.random()-0.5]).reshape(3,1)*fact
    print(f"{step = }")

    q2 = q + step
    print(f"{q2 =}")
    q2 = (q2+np.pi) % (2*np.pi) - np.pi
    print(f"{q2 = }")
    fz2 = fz_j1_j2_j3(q2[0],q2[1],q2[2])
    fy2 = fy_j1_j2_j3(q2[0],q2[1],q2[2])
    err = (fz2-fz_goal).T@(fz2-fz_goal)+(fy2-fy_goal).T@(fy2-fy_goal)
    print(f"1 {err= }")
    # print(f"{prev_err= }")
    if err < prev_err:
        q=q2
        q*180/np.pi
        print(f"{q= }")
        fz=fz2
        fy=fy2
        prev_err=err
        
        q2 = q2 + step
        fz2 = fz_j1_j2_j3(q2[0],q2[1],q2[2])
        fy2 = fy_j1_j2_j3(q2[0],q2[1],q2[2])
        err = (fz2-fz_goal).T@(fz2-fz_goal)+(fy2-fy_goal).T@(fy2-fy_goal)
        print(f"2{err= }")
        while err < prev_err:
           fails=0
           q=q2
           q*180/np.pi
           print(f"{q=}")
           fz=fz2
           fy=fy2
           prev_err=err
           print(f"{prev_err=}")
        
           q2 = q2 + step   
           fz2 = fz_j1_j2_j3(q2[0],q2[1],q2[2])
           fy2 = fy_j1_j2_j3(q2[0],q2[1],q2[2])
           err = (fz2-fz_goal).T@(fz2-fz_goal)+(fy2-fy_goal).T@(fy2-fy_goal);
           err2=err
           print(f"{err2= }")
        
    else:
        fails=fails+1
        if fails>2:
            fact=fact*0.5
            fails=0
        
    
   
        
q_goal = q 
print(f"{q_goal = }")   