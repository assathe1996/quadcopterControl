import re
f = open("state.txt", "r")
base = 57704000.000000

while(f.readline() != ""):
	data = f.readline().split(" ")
	#timestamp = re.findall(r'\d+', data[3])[0]
	#timestamp = (float(timestamp) - base) * 1e-6
	#print(timestamp)
	print(-1* (float(data[4]) + 2))