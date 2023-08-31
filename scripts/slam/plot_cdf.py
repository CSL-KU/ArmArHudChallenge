import numpy as np
import matplotlib.pyplot as plt

def split(sequence, sep):
    arr = []
    for val in sequence:
        if val == sep:
            yield arr
            arr = []
        else:
            arr.append(val)
    yield arr
    
def cdfPlot(data, num_bins, axs, row, col, label):
    np_arr = np.array(data).astype(float)
    histo, bins_count = np.histogram(np_arr, bins=num_bins)
    pdf = histo / sum(histo)
    cdf = np.cumsum(pdf)
    
    if row > -1:
        axs[row, col].plot(bins_count[1:], cdf)
        axs[row, col].set_title("{} ({} times)".format(label, len(data)))
        axs[row, col].set_xlabel("Execution time (ms)")
        axs[row, col].set_ylabel("CDF")
    else:
        axs[col].plot(bins_count[1:], cdf)
        axs[col].set_title("{} ({} times)".format(label, len(data)))
        axs[col].set_xlabel("Execution time (ms)")
        axs[col].set_ylabel("CDF")

for str in ["solo", "corun"]:
    for i in range(1,6):
        VFE_file = open("{}/{}/times/VFE_times.txt".format(i, str), 'r')
        VFE_times = list(split([line.strip() for line in VFE_file.readlines()], ''))
        fig, ax = plt.subplots(2,3)    
        cdfPlot(VFE_times[0][1:], 100, ax, 0, 0, "Image Pre-Processing")
        cdfPlot(VFE_times[1][1:], 100, ax, 0, 1, "Keypoints Tracking")
        cdfPlot(VFE_times[2][1:], 100, ax, 0, 2, "Outliers Filtering")
        cdfPlot(VFE_times[3][1:], 100, ax, 1, 0, "Pose Estimation")
        cdfPlot(VFE_times[4][1:], 100, ax, 1, 1, "Keyframe Creation")
        fig.suptitle("MH0{}-{}-Visual Front End".format(i, str.capitalize()), fontsize=28)
        fig.set_size_inches(18.5,10.5)
        plt.savefig("{}/{}/times/VFE_cdfs.pdf".format(i, str), bbox_inches='tight')
        plt.close()
        VFE_file.close()
        
        
        MAPPER_file = open("{}/{}/times/MAPPER_times.txt".format(i, str), 'r')
        MAPPER_times = list(split([line.strip() for line in MAPPER_file.readlines()], ''))
        fig, ax = plt.subplots(1,2)    
        cdfPlot(MAPPER_times[0][1:], 100, ax, -1, 0, "Triangulation")
        cdfPlot(MAPPER_times[1][1:], 100, ax, -1, 1, "Local Map Tracking")
        fig.suptitle("MH0{}-{}-Mapping".format(i, str.capitalize()), fontsize=28)
        fig.set_size_inches(18.5,10.5)
        plt.savefig("{}/{}/times/MAPPER_cdfs.pdf".format(i, str))
        plt.close()
        MAPPER_file.close()
        
        
        STATEOPT_file = open("{}/{}/times/STATEOPT_times.txt".format(i, str), 'r')
        STATEOPT_times = list(split([line.strip() for line in STATEOPT_file.readlines()], ''))
        fig, ax = plt.subplots(1,2)    
        cdfPlot(STATEOPT_times[0][1:], 100, ax, -1, 0, "Local Bundle Adjustment")
        cdfPlot(STATEOPT_times[1][1:], 100, ax, -1, 1, "Keyframes Filtering")
        fig.suptitle("MH0{}-{}-State Optimization".format(i, str.capitalize()), fontsize=28)
        fig.set_size_inches(18.5,10.5)
        plt.savefig("{}/{}/times/STATEOPT_cdfs.pdf".format(i, str))
        plt.close()
        STATEOPT_file.close()
        
        
        LOOPCLOSER_file = open("{}/{}/times/LOOPCLOSER_times.txt".format(i, str), 'r')
        LOOPCLOSER_times = list(split([line.strip() for line in LOOPCLOSER_file.readlines()], ''))
        fig, ax = plt.subplots(2,2)    
        cdfPlot(LOOPCLOSER_times[0][1:], 100, ax, 0, 0, "Online BoW")
        cdfPlot(LOOPCLOSER_times[1][1:], 100, ax, 0, 1, "Loop Candidate Processing")
        cdfPlot(LOOPCLOSER_times[2][1:], 100, ax, 1, 0, "Pose Graph Optimization")
        cdfPlot(LOOPCLOSER_times[3][1:], 100, ax, 1, 1, "Loose Bundle Adjustment")
        fig.suptitle("MH0{}-{}-Loop Closer".format(i, str.capitalize()), fontsize=28)
        fig.set_size_inches(18.5,10.5)
        plt.savefig("{}/{}/times/LOOPCLOSER_cdfs.pdf".format(i, str))
        plt.close()
        LOOPCLOSER_file.close()
    