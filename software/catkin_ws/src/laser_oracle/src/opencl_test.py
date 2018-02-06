#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import numpy as np
import array
import numpy as np
import pyopencl as cl
import time
import random
size=50000000
a=array.array('f',[random.random()]*size)
b=array.array('f',[random.random()]*size)
a_mv = memoryview(a)
b_mv = memoryview(b)

ctx = cl.create_some_context()
queue = cl.CommandQueue(ctx)

prg = cl.Program(ctx, """
__kernel void sum(
    __global const float *a_g, __global const float *b_g, __global float *res_g)
{
  int gid = get_global_id(0);
  res_g[gid] = a_g[gid] + b_g[gid];
}
""").build()

start=time.time()
mf = cl.mem_flags
a_buffer = cl.Buffer(ctx, mf.READ_ONLY | mf.COPY_HOST_PTR, hostbuf=a_mv)
b_buffer = cl.Buffer(ctx, mf.READ_ONLY | mf.COPY_HOST_PTR, hostbuf=b_mv)
#create result buffers localy and on device
res = np.zeros(len(a),np.float32)
res_buffer = cl.Buffer(ctx, mf.WRITE_ONLY, res.nbytes)
#run the program on hardware
prg.sum(queue, res.shape, None, a_buffer, b_buffer, res_buffer)
#copy data back off the device
cl.enqueue_copy(queue,res,res_buffer)
print(time.time()-start)
print(sum(res))

start=time.time()
vals=[a[i]+b[i] for i in range(0,size)]
print(time.time()-start)
print(sum(vals))
