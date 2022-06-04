import re
import math

orig = open("org.replay", "r")
test = open("test.replay", "r")

origlines = orig.readlines()
testlines = test.readlines()

joints22 = [0]*22
joints24 = [0]*24

med22 = [0]*22
med24 = [0]*24

c = 0
for j in range(10000, 10100):
    
    org = re.findall("\(j .*\)", origlines[j])
    test = re.findall("\(j .*\)", testlines[j])

    if org:
        c +=1
        org_arr = org[0].rstrip(")").split(" ")[1:]
        test_arr = test[0].rstrip(")").split(" ")[1:]

        if len(org_arr) == 24:
            for i in range(len(org_arr)):
                
                try:
                    joints24[i] += abs((float(org_arr[i]) - float(test_arr[i])))/float(org_arr[i])
                    if i == 10:
                        print(j, abs((float(org_arr[i]) - float(test_arr[i])))/float(org_arr[i]))
                    
                except:
                    joints24[i] += abs((float(org_arr[i]) - float(test_arr[i])))
                    if i == 10:
                        print(j, abs((float(org_arr[i]) - float(test_arr[i]))))
        else:
            for i in range(len(org_arr)):
                
                try:
                    joints22[i] += abs((float(org_arr[i]) - float(test_arr[i])))/float(org_arr[i])
                    if i == 10:
                        print(j, abs((float(org_arr[i]) - float(test_arr[i])))/float(org_arr[i]))
                except:
                    joints22[i] += abs((float(org_arr[i]) - float(test_arr[i])))
                    if i == 10:
                        print(j, abs((float(org_arr[i]) - float(test_arr[i]))))

# for i  in range(len(joints22)):
#     print(f"{i+1} {joints22[i]/c}")
#     med22[i] = joints22[i]/c


# for i  in range(len(joints24)):
#     print(f"{i+1} {joints24[i]/c}")

# for i in range(9756, len(testlines)):
    
#     org = re.findall("\(j .*\)", origlines[i])
#     test = re.findall("\(j .*\)", testlines[i])

#     if org:
#         c +=1
#         org_arr = org[0].rstrip(")").split(" ")[1:]
#         test_arr = test[0].rstrip(")").split(" ")[1:]

#         if len(org_arr) == 22:
#             for i in range(len(org_arr)):
                
#                 try:
#                     joints22[i] += math.square((float(org_arr[i]) - float(test_arr[i]))/float(org_arr[i]) - med22[i])
                    
#                 except:
#                     joints22[i] += math.square((float(org_arr[i]) - float(test_arr[i])) - med22[i])