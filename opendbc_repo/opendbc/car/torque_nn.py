from __future__ import annotations

import json
import os
from difflib import SequenceMatcher
from functools import cache

import numpy as np

from opendbc.car.common.basedir import BASEDIR

NEURAL_PARAMS_PATH = os.path.join(BASEDIR, "torque_data", "neural_ff_weights.json")
TORQUE_NN_MODEL_PATH = os.path.join(BASEDIR, "torque_data", "lat_models")


def similarity(s1: str, s2: str) -> float:
  return SequenceMatcher(None, s1, s2).ratio()


class FluxModel:
  activation_function_names = {"σ": "sigmoid"}

  def __init__(self, params_file, zero_bias=False):
    with open(params_file) as f:
      params = json.load(f)

    self.input_size = params["input_size"]
    self.output_size = params["output_size"]
    self.input_mean = np.array(params["input_mean"], dtype=np.float32).T
    self.input_std = np.array(params["input_std"], dtype=np.float32).T
    self.layers = []
    self.friction_override = False

    for layer_params in params["layers"]:
      W = np.array(layer_params[next(key for key in layer_params if key.endswith("_W"))], dtype=np.float32).T
      b = np.array(layer_params[next(key for key in layer_params if key.endswith("_b"))], dtype=np.float32).T
      if zero_bias:
        b = np.zeros_like(b)
      activation = layer_params["activation"]
      for old_name, new_name in self.activation_function_names.items():
        activation = activation.replace(old_name, new_name)
      self.layers.append((W, b, activation))

    self.validate_layers()
    self.check_for_friction_override()

  @staticmethod
  def sigmoid(x):
    return 1 / (1 + np.exp(-x))

  @staticmethod
  def identity(x):
    return x

  def forward(self, x):
    for W, b, activation in self.layers:
      x = getattr(self, activation)(x.dot(W) + b)
    return x

  def evaluate(self, input_array):
    in_len = len(input_array)
    if in_len != self.input_size:
      if 2 <= in_len:
        input_array = input_array + [0] * (self.input_size - in_len)
      else:
        raise ValueError(f"Input array length {len(input_array)} must be length 2 or greater")

    input_array = np.array(input_array, dtype=np.float32)
    input_array = (input_array - self.input_mean) / self.input_std
    output_array = self.forward(input_array)
    return float(output_array[0, 0])

  def validate_layers(self):
    for W, b, activation in self.layers:
      if not hasattr(self, activation):
        raise ValueError(f"Unknown activation: {activation}")

  def check_for_friction_override(self):
    self.friction_override = self.evaluate([10.0, 0.0, 0.2]) < 0.1


class NanoFFModel:
  def __init__(self, weights_loc: str, platform: str):
    self.weights_loc = weights_loc
    self.platform = platform
    self.load_weights(platform)

  def load_weights(self, platform: str):
    with open(self.weights_loc) as f:
      self.weights = {key: np.array(value) for key, value in json.load(f)[platform].items()}

  @staticmethod
  def relu(x: np.ndarray):
    return np.maximum(0.0, x)

  def forward(self, x: np.ndarray):
    assert x.ndim == 1
    x = (x - self.weights["input_norm_mat"][:, 0]) / (
      self.weights["input_norm_mat"][:, 1] - self.weights["input_norm_mat"][:, 0]
    )
    x = self.relu(np.dot(x, self.weights["w_1"]) + self.weights["b_1"])
    x = self.relu(np.dot(x, self.weights["w_2"]) + self.weights["b_2"])
    x = self.relu(np.dot(x, self.weights["w_3"]) + self.weights["b_3"])
    return np.dot(x, self.weights["w_4"]) + self.weights["b_4"]

  def predict(self, x: list[float], do_sample: bool = False):
    x = self.forward(np.array(x))
    if do_sample:
      prediction = np.random.laplace(x[0], np.exp(x[1]) / self.weights["temperature"])
    else:
      prediction = x[0]
    prediction = prediction * (self.weights["output_norm_mat"][1] - self.weights["output_norm_mat"][0]) + self.weights["output_norm_mat"][0]
    return prediction


@cache
def get_nano_ff_platforms() -> frozenset[str]:
  try:
    with open(NEURAL_PARAMS_PATH) as f:
      return frozenset(json.load(f).keys())
  except (OSError, ValueError, TypeError):
    return frozenset()


def get_nn_model_path(car, eps_firmware) -> str | None:
  def check_nn_path(check_model):
    model_path = None
    max_similarity = -1.0
    for filename in os.listdir(TORQUE_NN_MODEL_PATH):
      if filename.endswith(".json"):
        model = filename.removesuffix(".json")
        similarity_score = similarity(model, check_model)
        if similarity_score > max_similarity:
          max_similarity = similarity_score
          model_path = os.path.join(TORQUE_NN_MODEL_PATH, filename)
    return model_path, max_similarity

  print("########get_nn_model_path :", car, eps_firmware)
  if len(eps_firmware) > 3:
    eps_firmware = eps_firmware.replace("\\", "")
    check_model = f"{car} {eps_firmware}"
  else:
    check_model = car

  model_path, max_similarity = check_nn_path(check_model)
  if model_path is None or car not in model_path or 0.0 <= max_similarity < 0.9:
    model_path, max_similarity = check_nn_path(car)
    if model_path is None or car not in model_path or 0.0 <= max_similarity < 0.9:
      model_path = None
  return model_path


def get_nn_model(car, eps_firmware) -> FluxModel | None:
  model_path = get_nn_model_path(car, eps_firmware)
  return FluxModel(model_path) if model_path is not None else None
