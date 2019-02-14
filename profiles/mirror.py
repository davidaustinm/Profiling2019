import sys
filename = sys.argv[1]
input = open(filename)
output = open('right-'+filename, 'w')
lines = input.readlines()

for line in lines:
    line = line.strip()
    fields = line.split(',')
    fields[1] = 27*12 - float(fields[1])
    output.write(fields[0] + ", " + str(fields[1]) + "\n")
output.close()
input.close()

filename = filename[:-3]
newfilename = filename+"velocities.csv"
input = open(newfilename)
output = open('right-'+newfilename, 'w')
lines = input.readlines()
for line in lines:
    output.write(line)
output.close()
input.close()

