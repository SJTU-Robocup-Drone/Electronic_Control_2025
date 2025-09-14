import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np
import rospy

class TensorRTModel:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        
        self.cuda_context = cuda.Device(0).make_context()
        self.cuda_context.push()
        
        # 加载引擎
        with open(engine_path, 'rb') as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        # 分配GPU内存
        self.inputs = []
        self.outputs = []
        self.bindings = []

        for binding in self.engine:
            binding_idx = self.engine.get_binding_index(binding)
            size = trt.volume(self.context.get_binding_shape(binding_idx))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))

            # 分配host和device内存
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            self.bindings.append(int(device_mem))

            if self.engine.binding_is_input(binding):
                self.inputs.append({'host': host_mem, 'device': device_mem})
            else:
                self.outputs.append({'host': host_mem, 'device': device_mem})

    def cleanup(self):
        if self.cuda_context:
            self.cuda_context.pop()
            

    def infer(self, input_data):
        self.cuda_context.push()
        # 复制输入数据到GPU
        
        np.copyto(self.inputs[0]['host'], input_data.ravel())
        cuda.memcpy_htod(self.inputs[0]['device'], self.inputs[0]['host'])

        # 执行推理
        self.context.execute_v2(bindings=self.bindings)

        # 复制输出数据到CPU
        cuda.memcpy_dtoh(self.outputs[0]['host'], self.outputs[0]['device'])
        
        self.cuda_context.pop()

        return self.outputs[0]['host']