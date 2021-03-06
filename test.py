import numpy as np
import MDAnalysis as mda
import math

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
        i = np.where(abs(np.real(l) - 1.0) < 1e-6)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        direction = np.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = np.linalg.eig(R)
        i = np.where(abs(np.real(l) - 1.0) < 1e-6)[0]
        if not len(i):
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


"""
b -2.287 6.82 0.023 0.441 0.623 -0.103 -0.638
l 1 0x0 -14.503 0.332 0.352 -0.971 -0 0.001 -0.24 (j -0.02 -2.16 -50 -0 0 0 -50 -0 0 0 -0 0 30 -60 30 0 0 0 -0 30 -60 30 -0 0)
l 2 0x0 -4.095 2.703 0.346 -0.149 0.006 0.011 -0.989 (j -96.64 -6.05 -87.31 -14.29 66 82.71 -92.02 20.73 -87.01 -59.83 -1.43 -4.33 21.26 -73.83 53.51 4.72 -1.62 3.25 27.23 -64.65 36.2 -2.79)
l 3 0x0 -1.446 5.875 0.353 0.187 0.045 0.008 -0.981 (j -64.7 -19.91 -110.25 -14.3 88.88 60.08 -69.15 20.71 -64.12 -82.56 -1.71 1.48 21 -74.88 57.45 -1.38 -0 -1.41 -1.56 26.78 -60.35 39.91 1.49 -0)
l 4 0x0 -1.886 -10.57 0.357 -0.468 -0.013 -0.003 -0.884 (j 71.63 -0.33 -78.35 -20 56.98 90.96 -101.21 15 -95.99 -50.13 0.27 -5.61 16.86 -56.44 38.66 3.72 0.44 0.98 31.61 -73.74 41.22 -2.88)
l 5 0x0 -8.556 3.774 0.349 -0.945 -0.059 -0.022 -0.321 (j -17.27 0.65 -105.78 -20.07 84.24 64.69 -73.55 14.93 -68.75 -72.69 0.13 0.05 28.05 -66.93 39.13 -4.29 0.37 13.32 23.6 -70.44 47.56 -16.9)
l 6 0x0 3.895 3.787 0.362 -0.959 0.046 0.014 -0.28 (j 85.92 2.98 -105.06 -15 91.3 55 -73.75 20 -61.68 -87.63 -2.36 -11.53 8.6 -60.59 48.66 13.98 -2.4 16.09 27.6 -54.07 24.87 -13.58)
l 7 0x0 0.958 0.894 0.353 -0.137 0.053 -0.005 -0.989 (j 74.55 0.02 -63.85 -15.33 59.82 64.85 -73.01 19.67 -68.89 -55.69 -0.41 1.53 14.42 -63.31 54.99 -1.5 0 -0.39 -1.49 35.04 -67.6 39.5 1.46 -0)
l 8 0x0 10.008 0.973 0.357 -0.831 -0.021 0.025 0.555 (j 77.85 0.13 -92.13 -19.9 79.94 69.46 -87.37 15.1 -73.03 -73.14 -22.33 -4.76 9.86 -62.24 39.86 0.66 -0 -21.18 1.54 15.49 -74.63 47.76 -0.85 0)
l 9 0x0 4.992 4.622 0.355 -0.012 0.052 0.001 -0.999 (j -15.96 1.82 -107.81 -15.09 96.12 50.63 -69.15 19.9 -57.68 -91.01 -0.27 1.65 32.72 -72.93 46.45 -1.63 0 -0.37 -1.39 14.09 -56.56 48.32 1.37 0)
l 10 0x0 -4.79 5.457 0.356 -0.897 0.022 0.003 -0.441 (j -24.44 -8.55 -78.32 -15.1 73.31 68.19 -101.05 19.9 -79.7 -69.23 -0.04 -1.52 30.18 -81.33 52.16 3.33 0 -0.04 4.84 17.21 -57.57 41.19 -3.04 0)
l 11 0x0 -2.339 7.402 0.353 -0.958 0.026 0.002 0.284 (j -55.05 -44.95 -64.6 -15.42 59.56 87.71 -97.07 19.56 -91.5 -55.5 -3.28 1.45 31.94 -71.1 37.61 -0.47 -0 -2.16 0.17 7.98 -58.68 48.64 1.02 -0)
r 1 0x0 14.536 0.235 0.371 -0.167 0 -0.001 -0.986 (j 20.75 -45 -76.93 -15.6 70 15 -76.93 14.4 -70 -15 -0 -0.2 30.14 -70.37 40.25 0.11 -0 -0.2 30.07 -70.22 40.18 0.11)
r 2 0x0 12.122 0.548 0.345 -0.222 0.001 -0.004 -0.975 (j -45.89 -45 -81.15 -15.33 70 15 -81.15 14.67 -70 -15 -0.51 -5.29 28.91 -67.89 39.01 5.37 0 -0.51 4.51 33.51 -77.53 46.11 -4.43 0)
r 3 0x0 -0.568 7.527 0.356 -0.82 0.003 0.011 0.572 (j -2.37 -45 -80.12 -15.17 70 15 -80.12 14.83 -70 -15 -2.25 -3.43 20.99 -66.09 42.39 3.03 0 -2.25 4.35 15.04 -56.04 38.58 -4.67 0)
r 4 0x0 8.198 3.586 0.359 0.069 -0.019 0.002 -0.997 (j 27.28 -45 -80.15 -14.53 70 15 -80.14 15.47 -70 -15 -0.84 -1.45 7.97 -55.87 45 1.5 0 -0.84 0.24 22.84 -61.27 37.17 -0.18 0)
r 5 0x0 5.753 2.842 0.348 -0.676 -0.018 -0.002 0.737 (j -17.72 -45 -79.9 -15.42 70 15 -79.9 14.58 -70 -15 -21.28 -6.08 14.98 -67 39.75 2.27 0 -21.31 7.84 19.84 -75.7 44.95 -4.02 0)
r 6 0x0 2.285 0.97 0.343 -0.729 0.003 0.004 0.684 (j -82.84 -45 -79.93 -15.41 70 15 -79.92 14.59 -70 -15 -10.92 -5.69 20.45 -67.55 38.85 4.91 0 -10.92 9.39 19.74 -75.12 47.97 -8.59 0)
r 7 0x0 7.78 0.887 0.375 -0.644 -0.017 0.018 -0.765 (j 39.65 -45 -60.09 -13.98 70 15 -60.09 16.01 -70 -15 -2.09 -1.01 18.46 -72.6 48.4 1.03 -2.09 0.22 32.21 -73.74 38.18 -0.11)
r 8 0x0 -1.116 0.953 0.378 -0.268 -0.001 0.001 -0.963 (j -6.86 -45 -78.39 -14.92 70 15 -78.39 15.08 -70 -15 -16.83 -1.64 25.65 -87.41 50.51 -0.48 -16.82 2 20.81 -76.71 44 -0.32)
r 9 0x0 1.795 6.193 0.386 0.068 -0.044 -0.005 -0.997 (j -25.87 -45 -72.69 -14.82 70 15 -72.69 15.18 -70 -15 -1.07 3.65 30.25 -72.85 39.62 -3.86 -1.07 -4.66 17.99 -77.77 55.24 4.47)
r 10 0x0 -2.392 6.774 0.094 0.183 0.727 0.203 -0.63 (j 12.75 -45 -73.83 -21.09 55.95 9.69 -71.68 24.51 -55.99 -9.71 -2.1 -4.03 26 -53.68 27.59 3.96 0 -1.88 -0.41 31.45 -59.01 28.08 0.77 0)
r 11 0x0 -3.067 3.001 0.354 0.2 -0.007 -0.017 -0.98 (j 70.98 -45 -79.87 -14.49 70 15 -79.87 15.51 -70 -15 -7.26 -8 20.46 -62.79 36.58 7.6 0 -6.86 9.32 8.51 -59.26 44.52 -9.02 0)
"""


# -3.85077e-08 -1 6.12551e-09 0, 1 -3.85077e-08 -2.8613e-07 0, 2.8613e-07 6.1255e-09 1 0, -14.2 9.2 0.396077 1

# 14.426 1.118 0.371, -0.436 -0 -0.001 -0.9
# j -2.77 -45 -76.9 -15.68 70 15 -76.9 14.32 -70 -15 -0 -0.22 30.11 -70.38 40.33 0.08 -0 -0.22 30.01 -70.15 40.21 0.08)
# 0.81996 0.572416 -0.00227088 0, -0.572419 0.81996 -0.00125617 0, 0.00114298 0.00232991 0.999997 0,W 14.426 1.11864 0.46143 1

# 0.81996 0.572416 -0.00227088 0, -0.40559 0.57818 -0.707958 0, -0.403933 0.581418 0.706251 0, 14.424 1.12169 0.524958 1
"""
r = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
r180 = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0 ,0 ,1]])
r90 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0 ,0 ,1]])
R2 = r90@r
#print(R2)
R1 = mda.lib.transformations.rotation_from_matrix(R2)
#print(R1)
q = mda.lib.transformations.quaternion_about_axis(R1[0], R1[1])
#print(q)


v = np.array([[0.81996, -0.572419, 0.001142, 0], [0.572416, 0.81996, 0.00232991, 0], [-0.00227088, -0.00125617, 0.999997, 0], [0, 0, 0, 1]])
u0 = v[0,0:3]
u1 = v[1,0:3]
u2 = v[2,0:3]

#print(v)
#print(np.linalg.norm(u0), np.linalg.norm(u1), np.linalg.norm(u2))
#print(u0@u1.T, u2@u0.T, u1@u2.T)
u1 = u1/np.linalg.norm(u1)
u2 = u2/np.linalg.norm(u2)
#print(u1, u2)
u01 = np.cross(u1,u2)
#print("u01", u01)

v[0,0:3] = u01
v[1,0:3] = u1
v[2,0:3] = u2

#print("V", v)

V1 = r90@v
V2 = mda.lib.transformations.rotation_from_matrix(V1)
q = mda.lib.transformations.quaternion_about_axis(V2[0], V2[1])
#print(q)

t = np.array([[0.81996, -0.40559, -0.403933, 0], [0.572416, 0.57818, 0.581418, 0], [-0.00227088, -0.707958, 0.706251, 0], [0, 0, 0, 1]])
u0 = t[0,0:3]
u1 = t[1,0:3]
u2 = t[2,0:3]

u1 = u1/np.linalg.norm(u1)
u2 = u2/np.linalg.norm(u2)

u01 = np.cross(u1,u2)

t[0,0:3] = u01
t[1,0:3] = u1
t[2,0:3] = u2

# Coord mundo -> coord robo
T1 = np.linalg.inv(v)@t

print("T1", T1)

T2 = mda.lib.transformations.rotation_from_matrix(T1)
q = mda.lib.transformations.quaternion_about_axis(T2[0], T2[1])

print(q)
e = mda.lib.transformations.euler_from_quaternion(q)
print(e[0]*180/3.14, e[1]*180/3.14, e[2]*180/3.14)
"""


# -----------------------------

r90z = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0 ,0 ,1]])
r90y = np.array([0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1]).reshape(4,4).T
r90x = np.array([1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0 ,0, 0, 0, 0, 1]).reshape(4,4).T
r90 = r90z

# l 1 0x0 -14.503 0.332 0.352, -0.971 -0 0.001 -0.24 
# (j -0.02 -2.16 -50 -0 0 0 -50 -0 0 0 -0 0 30 -60 30 0 0 0 -0 30 -60 30 -0 0)

# l 1 0x0 -14.444 -0.031 0.355 -0.999 0.011 0.028 0.038 
# (j 0.57 -4.74 -87.61 -14.99 82.17 59.72 -82.91 20.02 -70.83 -78.46 -0.01 -1.65 23.19 -80.56 54.61 3 0 0 3.94 15.42 -57.93 39.75 -2.58 0)

# 0.466764 -0.884382 -3.46012e-05 0 0.884381 0.466763 0.00186376 0 -0.00163213 -0.000900536 0.999998 0 -14.5029 0.331594 0.351977 1

# find quaternion

l = "(j 0.57 -4.74 -87.61 -14.99 82.17 59.72 -82.91 20.02 -70.83 -78.46 -0.01 -1.65 23.19 -80.56 54.61 3 0 0 3.94 15.42 -57.93 39.75 -2.58 0)"
l = l.split(" ")[1:]
for i in range(len(l)):
        print(f"{i+1}:{l[i]}")

s = "-0.0790043 -0.995768 0.046958 0 0.581992 -0.0843169 -0.808812 0 0.809348 -0.0365704 0.58619 0 -14.4252 -0.0963057 0.100552 1"
arr = [float(x) for x in s.split(" ")]

#r1 = np.array([0.466764, -0.884382, -3.46012e-05, 0, 0.884381, 0.466763, 0.00186376, 0, -0.00163213, -0.000900536, 0.999998, 0, -14.5029, 0.331594, 0.351977, 1]).reshape(4,4).T
r2 = np.array(arr).reshape(4,4).T

r = r2
#print(r)
u0 = r[0,0:3]
u1 = r[1,0:3]
u2 = r[2,0:3]

u1 = u1/np.linalg.norm(u1)
u2 = u2/np.linalg.norm(u2)
#print(u1, u2)
u01 = np.cross(u1,u2)
#print("u01", u01)

r[0,0:3] = u01
r[1,0:3] = u1
r[2,0:3] = u2

#print("V", v)

V1 = r90@r
V2 = rotation_from_matrix(V1)
#print(V2)
q = mda.lib.transformations.quaternion_about_axis(V2[0], V2[1])
print(f"Quaternion: {q}\n\n")

joints = [
        "-0.078691 -0.996889 -0.00456126 0 0.99685 -0.0787315 0.0095113 0 -0.00984082 -0.00379844 0.999944 0 -14.4632 -0.0945015 0.0363568 1",
]

for joint in joints:
    
    arr = [float(x) for x in joint.split(" ")]
    

    h = np.array(arr).reshape(4,4).T
    print(f"{h = }")
    u0 = h[0,0:3]
    u1 = h[1,0:3]
    u2 = h[2,0:3]

    u1 = u1/np.linalg.norm(u1)
    u2 = u2/np.linalg.norm(u2)

    u01 = np.cross(u1,u2)

    h[0,0:3] = u01
    h[1,0:3] = u1
    h[2,0:3] = u2

    # Coord mundo -> coord robo
    T1 = np.linalg.inv(r)@h

    T2 = rotation_from_matrix(T1)
    #print(np.around(T2[0]*180/np.pi,2))
    q = mda.lib.transformations.quaternion_about_axis(T2[0], T2[1])

    #print(q)
    e = mda.lib.transformations.euler_from_quaternion(q)
    print(np.around(e[0]*180/np.pi,2), np.around(e[1]*180/np.pi,2), np.around(e[2]*180/np.pi,2))





