"""TensorRT 엔진으로 YOLOPv2 의 두 출력만 낸다.

엔진은 검출 헤드를 뺀 그래프로 구웠다(ONNX 노드 433 -> 326). 노드는 원래
검출 출력을 버리고 있었으므로 기능 차이는 없다.

버퍼는 한 번만 잡고 계속 재사용한다. 출력 버퍼도 재사용하므로 호출자는
다음 infer 전에 값을 다 써야 한다. 노드의 _infer 는 바로 numpy 로 옮기므로
문제가 없다.

torch 텐서의 data_ptr 을 그대로 TensorRT 에 넘긴다. pycuda 가 필요 없고,
torch 의 현재 스트림을 그대로 쓰므로 동기화 규약도 한 곳으로 모인다.
"""
from __future__ import annotations

from pathlib import Path


class TensorRTLane:
    """차선/주행가능영역 두 출력을 내는 TensorRT 실행기."""

    def __init__(self, engine_path: str, torch) -> None:
        import tensorrt as trt

        path = Path(engine_path)
        if not path.is_file():
            raise FileNotFoundError(f'엔진이 없다: {engine_path}')

        self._trt = trt
        self._torch = torch
        logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(logger)
        engine = runtime.deserialize_cuda_engine(path.read_bytes())
        if engine is None:
            raise RuntimeError('엔진 역직렬화 실패')
        self._engine = engine
        self._context = engine.create_execution_context()

        to_torch = {
            trt.float32: torch.float32,
            trt.float16: torch.float16,
            trt.int32: torch.int32,
            trt.int8: torch.int8,
        }
        self._buffers = {}
        inputs, outputs = [], []
        for index in range(engine.num_io_tensors):
            name = engine.get_tensor_name(index)
            shape = tuple(engine.get_tensor_shape(name))
            dtype = engine.get_tensor_dtype(name)
            if dtype not in to_torch:
                raise RuntimeError(f'다루지 않는 자료형: {name} {dtype}')
            buffer = torch.empty(shape, dtype=to_torch[dtype], device='cuda')
            self._buffers[name] = buffer
            self._context.set_tensor_address(name, buffer.data_ptr())
            if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                inputs.append((name, shape))
            else:
                outputs.append((name, shape))

        if len(inputs) != 1 or len(outputs) != 2:
            raise RuntimeError(
                f'입출력 개수가 예상과 다르다: 입력 {len(inputs)} 출력 {len(outputs)}')

        # 이름은 ONNX 내보내기가 붙인 일련번호라 의미가 없다. 채널 수로 가린다.
        self.input_name = inputs[0][0]
        self.drive_name = next(n for n, s in outputs if s[1] == 2)
        self.lane_name = next(n for n, s in outputs if s[1] == 1)
        self.input_dtype = self._buffers[self.input_name].dtype
        self.description = (
            f'TensorRT {trt.__version__} '
            f'입력 {inputs[0][1]} {self.input_dtype} '
            f'출력 {self.drive_name}/{self.lane_name}')

        # 첫 호출에서 지연이 튀지 않게 미리 돌린다.
        warm = torch.zeros_like(self._buffers[self.input_name])
        for _ in range(3):
            self.infer(warm)

    def infer(self, tensor):
        """(drive, lane) 을 돌려준다. 둘 다 GPU 텐서이고 다음 호출에 덮인다."""
        target = self._buffers[self.input_name]
        source = tensor.reshape(target.shape)
        if source.data_ptr() != target.data_ptr():
            target.copy_(source)
        stream = self._torch.cuda.current_stream()
        if not self._context.execute_async_v3(stream.cuda_stream):
            raise RuntimeError('TensorRT 실행 실패')
        stream.synchronize()
        return self._buffers[self.drive_name], self._buffers[self.lane_name]
