"""
Author: Sippawit Thammawiset
Date: September 8, 2024.
File: lane_detector.py
"""

from typing import Any, Union
import numpy as np
import onnxruntime


class LaneDetector:
    def __init__(self,
                 model_path: str | Any,
                 input_name: str | Any,
                 output_name: Union[list[str], Any],
                 providers: Union[str, Any] = None) -> None:
        self.model_path = model_path
        self.input_name = input_name
        self.input_shape = None
        self.output_name = output_name
        self.output_shape = None

        if providers is None:
            self.providers = ['CPUExecutionProvider']
        else:
            self.providers = providers

        self.__load_model()

    def __load_model(self) -> None:
        try:
            self.session = onnxruntime.InferenceSession(
                path_or_bytes=self.model_path,
                providers=self.providers
            )
        except Exception:
            raise FileNotFoundError(
                f'The ONNX model file could not be found. '
                f'Received: model_path={self.model_path}'
            )

        self.input_shape = self.session.get_inputs()[0].shape
        self.output_shape = self.session.get_outputs()[0].shape

    def __call__(self, X: np.ndarray) -> (np.ndarray, np.ndarray):
        if type(X) is not np.ndarray:
            raise TypeError(
                f'Expected \'X\' argument as type {np.ndarray}. '
                f'Received: {type(X)}'
            )

        if len(X.shape) != 4:
            raise ValueError(
                f'Expected 4-rank with (batch_size, height, width, channels) for {self.input_name}. '
                f'Received: {X.shape}'
            )

        if X.shape[3] != 3:
            raise ValueError(
                f'Expected 3 channels for {self.input_name} (RGB image). '
                f'Received: {X.shape[3]}'
            )

        if X.shape[1] != self.input_shape[1] or X.shape[2] != self.input_shape[2]:
            raise ValueError(
                f'Expected shape with {self.input_shape} for {self.input_name}. '
                f'Received: {X.shape}'
            )

        ortvalue = onnxruntime.OrtValue.ortvalue_from_numpy(X.astype('float32'))
        y1_pred, y2_pred = self.session.run(self.output_name, {self.input_name: ortvalue})

        return y1_pred, y2_pred
