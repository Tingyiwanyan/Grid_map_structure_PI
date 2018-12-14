import numpy as np
import numpy as np
import matplotlib.pyplot as plt

text_file = open("reachability.txt")
lines = text_file.read().split(',')
lines.remove('')
lines2 = [float(lines[i]) for i in range(len(lines))]
reachability = np.zeros((30,30))
max_value = max(lines2)
least_value = sorted(lines2)[1]

for i in range(30):
    for j in range(30):
        reachability[i, j] = lines2[30*i+j]/(np.abs(max_value))



plt.imshow(reachability)
plt.show()
