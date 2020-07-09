import math

fin = []
for i in range(2):
    arm, angle = input("Enter arm " + str(i + 1) + "'s length and relative angle in degrees:").split()
    fin.append([int(arm), math.radians(int(angle))])

pos1 = [fin[0][0]*math.cos(fin[0][1]), fin[0][0]*math.sin(fin[0][1])]
pos2 = [fin[1][0]*math.cos(fin[1][1] + fin[0][1] - math.pi) + pos1[0], fin[1][0]*math.sin(fin[1][1] + fin[0][1] - math.pi) + pos1[1]]
print("The coordinates of the hand are: (" + str(pos2[0]) + "," + str(pos2[1]) + ")")
