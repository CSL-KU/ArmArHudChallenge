import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

error_arrays = []
label_names = []

error_arrays.append(np.load("Solo.npz"))
label_names.append("Solo")

error_arrays.append(np.load("BwReadLLC.npz"))
label_names.append("BwRead(LLC)")

error_arrays.append(np.load("BwReadDRAM.npz"))
label_names.append("BwRead(DRAM)")

error_arrays.append(np.load("BwWriteLLC.npz"))
label_names.append("BwWrite(LLC)")

error_arrays.append(np.load("BwWriteDRAM.npz"))
label_names.append("BwWrite(DRAM)")

error_arrays.append(np.load("PLLReadLLC.npz"))
label_names.append("PLLRead(LLC)")

error_arrays.append(np.load("PLLReadDRAM.npz"))
label_names.append("PLLRead(DRAM)")

error_arrays.append(np.load("PLLWriteLLC.npz"))
label_names.append("PLLWrite(LLC)")

error_arrays.append(np.load("PLLWriteDRAM.npz"))
label_names.append("PLLWrite(DRAM)")

# error_arrays.append(np.load("BkBwReadLLC.npz"))
# label_names.append("BkBwRead(LLC)")

# error_arrays.append(np.load("BkBwWriteLLC.npz"))
# label_names.append("BkBwWrite(LLC)")

error_arrays.append(np.load("BkPLLReadLLC.npz"))
label_names.append("BkPLLRead(LLC)")

error_arrays.append(np.load("BkPLLWriteLLC.npz"))
label_names.append("BkPLLWrite(LLC)")

box = sns.boxplot(data=error_arrays, showfliers=False)
box.set_xticklabels(label_names)
box.set_ylabel("ATE (m)", fontsize=14)

box.tick_params(axis='x', rotation=-45)

fig = plt.gcf()
fig.set_size_inches(15, 9)

#plt.show()
plt.savefig("boxes.pdf", dpi=100, bbox_inches='tight')