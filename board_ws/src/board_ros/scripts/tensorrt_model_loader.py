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
        
        # ��������
        with open(engine_path, 'rb') as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        # ����GPU�ڴ�
        self.inputs = []
        self.outputs = []
        self.bindings = []

        for binding in self.engine:
            binding_idx = self.engine.get_binding_index(binding)
            size = trt.volume(self.context.get_binding_shape(binding_idx))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))

            # ����host��device�ڴ�
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
        # �����������ݵ�GPU
        
        np.copyto(self.inputs[0]['host'], input_data.ravel())
        cuda.memcpy_htod(self.inputs[0]['device'], self.inputs[0]['host'])

        # ִ������
        self.context.execute_v2(bindings=self.bindings)

        # ����������ݵ�CPU
        cuda.memcpy_dtoh(self.outputs[0]['host'], self.outputs[0]['device'])
        
        self.cuda_context.pop()

        return self.outputs[0]['host']