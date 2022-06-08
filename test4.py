import numpy as np
import MDAnalysis as mda
import math
import json

def rotation_from_matrix(matrix):
        """Return rotation angle and axis from rotation matrix.

        >>> angle = (random.random() - 0.5) * (2*math.pi)
        >>> direc = np.random.random(3) - 0.5
        >>> point = np.random.random(3) - 0.5
        >>> R0 = rotation_matrix(angle, direc, point)
        >>> angle, direc, point = rotation_from_matrix(R0)
        >>> R1 = rotation_matrix(angle, direc, point)
        >>> is_same_transform(R0, R1)
        True

        """
        R = np.array(matrix, dtype=np.float64, copy=False)
        R33 = R[:3, :3]
        # direction: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = np.linalg.eig(R33.T)
        #print(l)
        i = np.where(abs(np.real(l) - 1.0) < 1e-6)[0]
        if not len(i):
            print("1")
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        direction = np.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = np.linalg.eig(R)
        i = np.where(abs(np.real(l) - 1.0) < 1e-6)[0]
        if not len(i):
            print("2")
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        point = np.real(Q[:, i[-1]]).squeeze()
        point /= point[3]
        # rotation angle depending on direction
        cosa = (np.trace(R33) - 1.0) / 2.0
        if abs(direction[2]) > 1e-8:
            sina = (R[1, 0] + (cosa - 1.0) * direction[0] * direction[1]) / direction[2]
        elif abs(direction[1]) > 1e-8:
            sina = (R[0, 2] + (cosa - 1.0) * direction[0] * direction[2]) / direction[1]
        else:
            sina = (R[2, 1] + (cosa - 1.0) * direction[1] * direction[2]) / direction[0]
        angle = math.atan2(sina, cosa)
        return angle, direction, point


def rot_x(val):
    return np.array([1, 0, 0, 0, 0, np.cos(val), -np.sin(val), 0, 0, np.sin(val), np.cos(val), 0, 0, 0, 0, 1]).reshape(4,4)

def rot_y(val):
    return np.array([np.cos(val), 0, np.sin(val), 0, 0, 1, 0, 0, -np.sin(val), 0, np.cos(val), 0, 0, 0, 0, 1]).reshape(4,4)

def rot_z(val):
    return np.array([np.cos(val), -np.sin(val), 0, 0, np.sin(val), np.cos(val), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]).reshape(4,4)



#print(rot_y(-np.pi/4))
#print(rot_x(np.pi/4))
#print(rot_z(np.pi/4))


def test(h, r):
    
    r90z = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0 ,0 ,1]])
    r90y = np.array([0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1]).reshape(4,4).T
    r90x = np.array([1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0 ,0, 0, 0, 0, 1]).reshape(4,4).T

    #r = r90y@r

    #h = rot_x(-np.pi/4)@h
    #h = rot_y(np.pi/2)@h
    #h = rot_z(np.pi/2)@h

    #r = rot_x(-np.pi/4)@r
    #r = rot_y(np.pi/4)@r
    #r = rot_z(np.pi/4)@r

    #print(h)

    u0 = h[0,0:3]
    u1 = h[1,0:3]
    u2 = h[2,0:3]

    u1 = u1/np.linalg.norm(u1)
    u2 = u2/np.linalg.norm(u2)

    u01 = np.cross(u1,u2)

    #print(np.linalg.norm(u01), np.linalg.norm(u1), np.linalg.norm(u2))

    h[0,0:3] = u01
    h[1,0:3] = u1
    h[2,0:3] = u2

    # Coord mundo -> coord robo
    #r = r90z@r
    #h = r90y@h
    T1 = np.linalg.inv(r)@h
    #T1 = rot_y(-np.pi/4)@rot_z(np.pi/4)@h

    u0 = T1[0,0:3]
    u1 = T1[1,0:3]
    u2 = T1[2,0:3]

    u1 = u1/np.linalg.norm(u1)
    u2 = u2/np.linalg.norm(u2)

    u01 = u0/np.linalg.norm(u0)

    T1[0,0:3] = u01
    T1[1,0:3] = u1
    T1[2,0:3] = u2

    #print(np.linalg.norm(u01), np.linalg.norm(u1), np.linalg.norm(u2))
    #print(T1)

    T2 = rotation_from_matrix(T1)
    #print(np.around(T2[0]*180/np.pi,2))
    q = mda.lib.transformations.quaternion_about_axis(T2[0], T2[1])

    #print(q)
    e = mda.lib.transformations.euler_from_quaternion(q)
    print(np.around(e[0]*180/np.pi,2), np.around(e[1]*180/np.pi,2), np.around(e[2]*180/np.pi,2))
    return T1


FILE = "samples/sample1-L3.json"


f = open(FILE, "r")
data = json.load(f)


with open("samples/reps.json", "r") as repfile: 
    repdata = json.load(repfile)


tmp_s = "-1 3.82137e-15 8.74228e-08 -0 8.74228e-08 4.37114e-08 1 0 0 1 -4.37114e-08 0 0 0 0 1"
arr = [float(x) for x in tmp_s.split(" ")]
tmp = np.array(arr).reshape(4,4).T

print(repdata[2])
rep = [float(x) for x in repdata[2].split(" ")]
#l = rep.split(" ")
for i in range(len(rep)):
    if i+1 in [11,12,13,14,15,16]:
        print(f"{i+1}:{rep[i]}")

#print(rot_x(np.pi/2))

body =  data['body']
thigh = data['thigh']
shank = data['shank']
foot =  data['foot']
shoulder = data['shoulder']
arm = data['arm']
head = data['head']


#b_arr = [float(x) for x in body.split(" ")]
#t_arr = [float(x) for x in thigh.split(" ")#]
#s_arr = [float(x) for x in shank.split(" ")#]
#f_arr = [float(x) for x in foot.split(" ")]#

#sh_arr =[float(x) for x in shoulder.split(" ")]
#ar_arr =[float(x) for x in arm.split(" ")]
#h_arr = [float(x) for x in head.split(" ")]

b = np.array(body).reshape(4,4).T
# print(b)

# -0.10842   0.990829   -0.0806452 -14.5057 
# -0.994001  -0.109226  -0.00563375 5.47707e-05 
# -0.0143906 0.0795506  0.996727    0.356331
# 0          0          0           1
t = np.array(thigh).reshape(4,4).T
s = np.array(shank).reshape(4,4).T
f = np.array(foot).reshape(4,4).T
sh= np.array(shoulder).reshape(4,4).T
ar= np.array(arm).reshape(4,4).T
h = np.array(head).reshape(4,4).T

#b = np.linalg.inv(tmp)@b

# print(5*"-")
# test(t, rot_z(np.pi/2)@b)
# test(t, rot_y(np.pi/2)@b)
# test(t, rot_x(np.pi/2)@b)

print(5*"-")
test(t, b) # 13?, 11?
test(s, b) # 12
test(f, b)

print(5*"-")
test(t, rot_z(np.pi/4)@b)
test(rot_z(np.pi/4)@t, b)
print(5*"-")
test(t, rot_x(-np.pi/4)@b)
test(rot_x(-np.pi/4)@t, b)
print(5*"-")
test(t, rot_x(-np.pi/4)@rot_z(np.pi/4)@b)
test(rot_x(-np.pi/4)@rot_z(np.pi/4)@t, b)
print(5*"-")
test(t, rot_z(np.pi/4)@rot_x(-np.pi/4)@b)
test(rot_z(np.pi/4)@rot_x(-np.pi/4)@t, b)
print(5*"-")
test(rot_z(np.pi/4)@t, rot_x(-np.pi/4)@b)
test(rot_x(-np.pi/4)@t, rot_z(np.pi/4)@b)

print(5*"-")
test(rot_z(np.pi/4)@t, rot_x(np.pi/4)@b)
test(rot_x(np.pi/4)@t, rot_z(np.pi/4)@b)

print(5*"-")
test(b, rot_z(np.pi/4)@t)
test(rot_z(np.pi/4)@b, t)
print(5*"-")
test(b, rot_x(-np.pi/4)@t)
test(rot_x(-np.pi/4)@b, t)
print(5*"-")
test(b, rot_x(-np.pi/4)@rot_z(np.pi/4)@t)
test(rot_x(-np.pi/4)@rot_z(np.pi/4)@b, t)
print(5*"-")
test(b, rot_z(np.pi/4)@rot_x(-np.pi/4)@t)
test(rot_z(np.pi/4)@rot_x(-np.pi/4)@b, t)
print(5*"-")
test(rot_z(np.pi/4)@b, rot_x(-np.pi/4)@t)
test(rot_x(-np.pi/4)@b, rot_z(np.pi/4)@t)

# print(5*"-")
# test(t, rot_y(np.pi/4)@b)
# test(rot_y(np.pi/4)@t, b)

# print(5*"-")
# test(t, rot_x(np.pi/4)@b)
# test(rot_x(np.pi/4)@t, b)

# print(5*"-")
# test(t, rot_x(-np.pi/4)@rot_z(np.pi/4)@b)
# test(t, rot_z(np.pi/4)@rot_x(-np.pi/4)@b)

# test(rot_x(-np.pi/4)@rot_z(np.pi/4)@t, b)
# test(rot_z(np.pi/4)@rot_x(-np.pi/4)@t, b)

#print(5*"-")
#test(t, h)
#test(s, h)
#test(f, h)
#
#print(5*"-")
#test(s,t) # 14
#test(f,t) # 16?
#
#print(5*"-")
#test(f,s) # 15
#test(t,s) 
#
#print(5*"-")
#test(t,f)
#test(s,f)

#test(t,np.linalg.inv(b)@b)
#test(f, b)
#test(t, rot_x(-np.pi/4)@rot_z(np.pi/4)@b)
#test(t, rot_z(np.pi/4)@rot_x(-np.pi/4)@b)
#
#print(rot_x(-np.pi/4)@rot_z(np.pi/4)@t)
#print(rot_z(np.pi/4)@rot_x(-np.pi/4)@t)
# print(5*"-")
#st = test(s, t) # 14
# ts = test(t, s)
# test(f, t) # 16
# test(f, s) # 15
# print(5*"-")
# x = test(f, st)
# test(st, f)
# test(f, ts)
# test(ts, f)
# test(b, x)
# test(x, b)

# print(5*"-")
# test(sh,b) # 3, 4
# test(ar,b)
# print(5*"-")
# test(ar, sh) # 5, 6
# test(sh, ar)


# for i in range(1, 5):

#     print(5*"-",i)

#     test(rot_x(np.pi/i)@np.linalg.inv(sh)@ar, b)
#     test(np.linalg.inv(rot_x(np.pi/i)@sh)@ar, b)
#     test(np.linalg.inv(sh)@rot_x(np.pi/i)@ar, b)
#     test(np.linalg.inv(sh)@ar, rot_x(np.pi/i)@b)

#     test(rot_y(np.pi/i)@np.linalg.inv(sh)@ar, b)
#     test(np.linalg.inv(rot_y(np.pi/i)@sh)@ar, b)
#     test(np.linalg.inv(sh)@rot_y(np.pi/i)@ar, b)
#     test(np.linalg.inv(sh)@ar, rot_y(np.pi/i)@b)

#     test(rot_z(np.pi/i)@np.linalg.inv(sh)@ar, b)
#     test(np.linalg.inv(rot_z(np.pi/i)@sh)@ar, b)
#     test(np.linalg.inv(sh)@rot_z(np.pi/i)@ar, b)
#     test(np.linalg.inv(sh)@ar, rot_z(np.pi/i)@b)

# test(ar, np.linalg.inv(sh)@b)
# 
# for i in range(1, 5):
# 
#     print(5*"-",i)
# 
#     test(rot_x(np.pi/i)@ar, np.linalg.inv(sh)@b)
#     test(ar, np.linalg.inv(rot_x(np.pi/i)@sh)@b)
#     test(ar, rot_x(np.pi/i)@np.linalg.inv(sh)@b)
#     test(ar, np.linalg.inv(sh)@rot_x(np.pi/i)@b)
# 
#     test(rot_y(np.pi/i)@ar, np.linalg.inv(sh)@b)
#     test(ar, np.linalg.inv(rot_y(np.pi/i)@sh)@b)
#     test(ar, rot_y(np.pi/i)@np.linalg.inv(sh)@b)
#     test(ar, np.linalg.inv(sh)@rot_y(np.pi/i)@b)
# 
#     test(rot_z(np.pi/i)@ar, np.linalg.inv(sh)@b)
#     test(ar, np.linalg.inv(rot_z(np.pi/i)@sh)@b)
#     test(ar, rot_z(np.pi/i)@np.linalg.inv(sh)@b)
#     test(ar, np.linalg.inv(sh)@rot_z(np.pi/i)@b)

#test(ar, np.linalg.inv(b)@sh)

# for i in range(1, 5):

#     print(5*"-",i)

#     test(rot_x(np.pi/i)@ar, np.linalg.inv(b)@sh)
#     test(ar, np.linalg.inv(rot_x(np.pi/i)@b)@sh)
#     test(ar, rot_x(np.pi/i)@np.linalg.inv(b)@sh)
#     test(ar, np.linalg.inv(b)@rot_x(np.pi/i)@sh)

#     test(rot_y(np.pi/i)@ar, np.linalg.inv(b)@sh)
#     test(ar, np.linalg.inv(rot_y(np.pi/i)@b)@sh)
#     test(ar, rot_y(np.pi/i)@np.linalg.inv(b)@sh)
#     test(ar, np.linalg.inv(b)@rot_y(np.pi/i)@sh)

#     test(rot_z(np.pi/i)@ar, np.linalg.inv(b)@sh)
#     test(ar, np.linalg.inv(rot_z(np.pi/i)@b)@sh)
#     test(ar, rot_z(np.pi/i)@np.linalg.inv(b)@sh)
#     test(ar, np.linalg.inv(b)@rot_z(np.pi/i)@sh)

# print(5*"-")
# test(np.linalg.inv(sh)@ar, shb)
# test(ar, shb)

#print("---------")
#test(ar, shh)
#
#bsh = test(b,sh)
#test(ar, sh) # 5

#test(shb, ar)
# test(sh, ar)
# test(ar, shb)
# test(shb, ar)
# test(ar, bsh)
# test(bsh, ar)

#test(sh, arb)
#test(arb, sh)


