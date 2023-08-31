import numpy as np
import matplotlib.pyplot as plt

# solo_file = open("cdf_timings/Solo_FrontEnd.txt", 'r')
# solo_file = open("cdf_timings/Solo_Mapper.txt", 'r')
# solo_file = open("cdf_timings/Solo_StateOpt.txt", 'r')
# temp = solo_file.readlines()[:-1]
# solo_times = np.array([x.strip() for x in temp], dtype='float32')

# print(len(solo_times))

# histo, bins_count = np.histogram(solo_times, bins=100)
# pdf = histo / sum(histo)
# cdf = np.cumsum(pdf)

# plt.plot(bins_count[1:], cdf, label="Solo")

# dos_file = open("1/dos-backup/times/FRONTEND_times.txt", 'r')
# dos_file = open("1/dos-backup/times/MAPPER_times.txt", 'r')         # DONE
# dos_file = open("1/dos-backup/times/STATEOPT_times.txt", 'r')
# dos_file = open("1/dos/times/FRONTEND_times.txt", 'r')              #DONE
# dos_file = open("1/dos/times/MAPPER_times.txt", 'r')
# dos_file = open("1/dos/times/STATEOPT_times.txt", 'r')

# dos_file = open("1/dnn/times/FRONTEND_times.txt", 'r')
# dos_file = open("1/dnn/times/MAPPER_times.txt", 'r')
# dos_file = open("1/solo/times/STATEOPT_times.txt", 'r')
# temp = dos_file.readlines()[:-1]
# dos_times = np.array([x.strip() for x in temp], dtype='float32')

# print(len(dos_times))

# Open Solo timing file
# solo_file = open("cdf_timings/Solo_FrontEnd.txt", 'r')
# solo_file = open("cdf_timings/Solo_Mapper.txt", 'r')
solo_file = open("cdf_timings/Solo_StateOpt.txt", 'r')

# Open DNN timing file
# dnn_file = open("cdf_timings/DNN_FrontEnd.txt", 'r')
# dnn_file = open("cdf_timings/DNN_Mapper.txt", 'r')
dnn_file = open("cdf_timings/DNN_StateOpt.txt", 'r')

# Open DoS timing file
# dos_file = open("cdf_timings/DoS_FrontEnd.txt", 'r')
# dos_file = open("cdf_timings/DoS_Mapper.txt", 'r')
dos_file = open("cdf_timings/DoS_StateOpt.txt", 'r')

# Get all timings into np arrays
temp = solo_file.readlines()[:-1]
solo_times = np.array([x.strip() for x in temp], dtype='float32')
temp = dnn_file.readlines()[:-1]
dnn_times = np.array([x.strip() for x in temp], dtype='float32')
temp = dos_file.readlines()[:-1]
dos_times = np.array([x.strip() for x in temp], dtype='float32')

# Plot Solo CDF
histo, bins_count = np.histogram(solo_times, bins=100)
pdf = histo / sum(histo)
cdf = np.cumsum(pdf)
plt.plot(bins_count[1:], cdf, label="Solo")

# Plot DNN CDF
histo, bins_count = np.histogram(dnn_times, bins=100)
pdf = histo / sum(histo)
cdf = np.cumsum(pdf)
plt.plot(bins_count[1:], cdf, label="DNN")

# Plot DoS CDF
histo, bins_count = np.histogram(dos_times, bins=100)
pdf = histo / sum(histo)
cdf = np.cumsum(pdf)
plt.plot(bins_count[1:], cdf, label="DoS")

plt.xlabel("Execution time (ms)")
plt.ylabel("CDF")
plt.legend()

#plt.show()
plt.savefig("cdf_timings/StateOptCDF.pdf", dpi=100, bbox_inches='tight')

