import torch
import stable_hopenetlite
import time
import numpy as np

deadline = 0.05

pos_net = stable_hopenetlite.shufflenet_v2_x1_0().cuda()
input=torch.randn(1,3,224,224).cuda()

first = 1
while(True):
    if first:
        pos_net(input)
        first = 0
    else:
        start = time.perf_counter()
        pos_net(input)
        end = time.perf_counter()
        
        dur = (end - start)
        if dur < deadline:
            time.sleep(deadline - dur)
