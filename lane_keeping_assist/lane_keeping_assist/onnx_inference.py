"""
Author: Sippawit Thammawiset
Date: September 9, 2024.
File: onnx_inference.py
"""

from typing import Any, Union, Optional
import numpy as np
import onnxruntime


class ONNXInference:
    def __init__(self,
                 model_filepath: str | Any,
                 input_name: str | Any,
                 output_name: Union[list[str], Any],
                 providers: Union[str, Any] = None,
                 name: Optional[str] = None) -> None:
        self.model_filepath = model_filepath
        self.input_name = input_name
        self.input_shape = None
        self.output_name = output_name
        self.output_shape = None

        if providers is None:
            self.providers = ['CPUExecutionProvider']
        else:
            self.providers = providers

        if name is None:
            self.name = 'ONNXInference'
        else:
            self.name = name

        self.__load_model()

    def __load_model(self) -> None:
        try:
            self.session = onnxruntime.InferenceSession(
                path_or_bytes=self.model_filepath,
                providers=self.providers
            )
        except Exception:
            raise FileNotFoundError(
                f'The ONNX model file could not be found. '
                f'Received: model_filepath={self.model_filepath}'
            )

        self.input_shape = self.session.get_inputs()[0].shape
        self.output_shape = self.session.get_outputs()[0].shape

    def predict(self, X: np.ndarray) -> (np.ndarray, np.ndarray):
        if type(X) is not np.ndarray:
            raise TypeError(
                f'Expected \'X\' argument as type {np.ndarray}. '
                f'Received: {type(X)}'
            )

        if len(X.shape) != 4:
            raise ValueError(
                f'Expected 4-rank tensor with (batch_size, height, width, channels) for {self.input_name}. '
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
        y_pred = self.session.run(self.output_name, {self.input_name: ortvalue})

        return y_pred

    def __str__(self):
        return self.name
