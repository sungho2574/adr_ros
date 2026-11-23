"""TensorRT execution for the exported mask encoder and recurrent policy."""
import json
from pathlib import Path

import numpy as np


class Engine:
    def __init__(self, path, stream):
        import pycuda.driver as cuda
        import tensorrt as trt

        self.cuda = cuda
        self.stream = stream
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(path.read_bytes())
        if self.engine is None:
            raise ValueError(f"Cannot load TensorRT engine: {path}")
        self.context = self.engine.create_execution_context()
        self.inputs, self.outputs, self.buffers = {}, {}, {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            host = cuda.pagelocked_empty(shape, dtype)
            device = cuda.mem_alloc(host.nbytes)
            self.buffers[name] = device
            self.context.set_tensor_address(name, int(device))
            tensors = (self.inputs if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
                       else self.outputs)
            tensors[name] = host

    def __call__(self, **inputs):
        if inputs.keys() != self.inputs.keys():
            raise ValueError(f"Expected TensorRT inputs {tuple(self.inputs)}")
        for name, value in inputs.items():
            np.copyto(self.inputs[name], value)
            self.cuda.memcpy_htod_async(self.buffers[name], self.inputs[name], self.stream)
        if not self.context.execute_async_v3(self.stream.handle):
            raise RuntimeError("TensorRT policy inference failed")
        for name, host in self.outputs.items():
            self.cuda.memcpy_dtoh_async(host, self.buffers[name], self.stream)
        self.stream.synchronize()
        return self.outputs


class TensorRTPolicy:
    device = "tensorrt"

    def __init__(self, bundle, engines, seed=0):
        import pycuda.autoinit

        self.meta = json.loads((Path(bundle) / "policy_meta.json").read_text())
        self.vec_keys, self.vec_dims = self.meta["enc_vec_keys"], self.meta["enc_vec_dims"]
        dims = self.meta["dims"]
        self.noise_shape = (1, dims["stoch"], dims["classes"])
        self.rng = np.random.default_rng(seed)
        stream = pycuda.autoinit.cuda.Stream()
        self.encoder = Engine(Path(engines) / "encoder.plan", stream)
        self.recurrent = Engine(Path(engines) / "step.plan", stream)
        self.reset()

    def reset(self):
        self.state = {name: np.zeros_like(self.recurrent.inputs[name])
                      for name in ("deter", "stoch", "previous")}
        self.mask_id = None

    def step(self, mask, vec, is_first, mask_id=None, gumbel=None):
        if is_first:
            self.reset()
        if mask_id is None or mask_id != self.mask_id:
            self.image = self.encoder(mask=np.asarray(mask).reshape(1, 64, 64))["image"]
            self.mask_id = mask_id
        if gumbel is None:
            gumbel = -np.log(-np.log(self.rng.uniform(1e-9, 1.0 - 1e-9, self.noise_shape)))
        values = self.recurrent(image=self.image, vector=np.asarray(vec).reshape(1, -1),
                                noise=gumbel, **self.state)
        self.state = {"deter": values["next_deter"], "stoch": values["next_stoch"],
                      "previous": values["action"]}
        return values["action"].copy(), values["gate"].copy()
