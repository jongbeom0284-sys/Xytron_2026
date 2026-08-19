from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class InferenceResult:
    lanes: list[np.ndarray]
    elapsed_ms: float


class CLRerNetEngine:
    """Load vendored CLRerNet and run inference directly from a BGR array."""

    def __init__(
        self,
        vendor_root: str | Path,
        config_path: str | Path,
        checkpoint_path: str | Path,
        device: str = "cuda:0",
        preprocess_size: tuple[int, int] = (1640, 590),
    ) -> None:
        self.vendor_root = Path(vendor_root).resolve()
        self.config_path = Path(config_path).resolve()
        self.checkpoint_path = Path(checkpoint_path).resolve()
        self.device = device
        self.preprocess_size = preprocess_size
        for path in (self.vendor_root, self.config_path, self.checkpoint_path):
            if not path.exists():
                raise FileNotFoundError(path)

        vendor_string = str(self.vendor_root)
        if vendor_string not in sys.path:
            sys.path.insert(0, vendor_string)

        import torch
        from libs.api.inference import get_prediction
        from libs.datasets.pipelines import Compose
        from mmdet.apis import init_detector

        self._torch = torch
        self._compose_type = Compose
        self._get_prediction = get_prediction
        # An explicit palette keeps MMDetection from constructing the CULane
        # evaluation dataset just to obtain visualization metadata.
        self._model = init_detector(
            str(self.config_path),
            str(self.checkpoint_path),
            device=self.device,
            palette="random",
        )
        self._model.bbox_head.test_cfg.as_lanes = False
        self._pipeline = self._compose_type(
            self._model.cfg.test_dataloader.dataset.pipeline
        )

    def _synchronize(self) -> None:
        if self.device.startswith("cuda"):
            self._torch.cuda.synchronize()

    def infer(self, source: np.ndarray) -> InferenceResult:
        if source.ndim != 3 or source.shape[2] != 3:
            raise ValueError("source must be a BGR image")

        source_height, source_width = source.shape[:2]
        virtual_width, virtual_height = self.preprocess_size
        virtual = cv2.resize(
            source,
            (virtual_width, virtual_height),
            interpolation=cv2.INTER_LINEAR,
        )
        data = {
            "filename": "<ros-image>",
            "sub_img_name": None,
            "img": virtual,
            "gt_points": [],
            "id_classes": [],
            "id_instances": [],
            "img_shape": virtual.shape,
            "ori_shape": virtual.shape,
        }
        packed = self._pipeline(data)
        model_input = {
            "inputs": [packed["inputs"]],
            "data_samples": [packed["data_samples"]],
        }

        self._synchronize()
        started = time.perf_counter()
        with self._torch.inference_mode():
            results = self._model.test_step(model_input)
        self._synchronize()
        elapsed_ms = (time.perf_counter() - started) * 1000.0

        virtual_lanes = self._get_prediction(
            results[0]["lanes"], virtual_height, virtual_width
        )
        lanes: list[np.ndarray] = []
        for lane in virtual_lanes:
            restored = np.asarray(lane, dtype=np.float32).reshape((-1, 2)).copy()
            restored[:, 0] *= source_width / virtual_width
            restored[:, 1] *= source_height / virtual_height
            lanes.append(restored)
        return InferenceResult(lanes=lanes, elapsed_ms=elapsed_ms)

    def warmup(self, image_shape: tuple[int, int, int] = (480, 640, 3)) -> None:
        self.infer(np.zeros(image_shape, dtype=np.uint8))
