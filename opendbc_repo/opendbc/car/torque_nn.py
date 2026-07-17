from __future__ import annotations

import json
import os
from difflib import SequenceMatcher
from functools import cache

import numpy as np

from opendbc.car.common.basedir import BASEDIR

EXPECTED_NNFF_INPUT_VARS = ("v_ego", "lateral_accel", "lateral_jerk", "roll",
  "lateral_accel_m03", "lateral_accel_m02", "lateral_accel_m01", "lateral_accel_p03", "lateral_accel_p06", "lateral_accel_p10", "lateral_accel_p15",
  "roll_m03", "roll_m02", "roll_m01", "roll_p03", "roll_p06", "roll_p10", "roll_p15")
NEURAL_PARAMS_PATH = os.path.join(BASEDIR, "torque_data", "neural_ff_weights.json")
TORQUE_NN_MODEL_PATH = os.path.join(BASEDIR, "torque_data", "lat_models")


def _normalize_name(value: str) -> str:
  return " ".join(value.replace("\\", "").replace("_", " ").replace("-", " ").upper().split())


def _similarity(first: str, second: str) -> float:
  return SequenceMatcher(None, first, second).ratio()


class FluxModel:
  activation_function_names = {"σ": "sigmoid"}

  def __init__(self, params_file: str, zero_bias: bool = False):
    with open(params_file, encoding="utf-8") as f:
      params = json.load(f)

    self.model_path = params_file
    self.input_size = int(params["input_size"])
    self.output_size = int(params["output_size"])
    self.input_vars = tuple(params.get("input_vars", []))

    if self.input_size != len(EXPECTED_NNFF_INPUT_VARS):
      raise ValueError(
        f"Unsupported NNFF input size in {params_file}: "
        f"{self.input_size}, expected {len(EXPECTED_NNFF_INPUT_VARS)}"
      )

    if self.input_vars and self.input_vars != EXPECTED_NNFF_INPUT_VARS:
      raise ValueError(
        f"Unsupported NNFF input layout in {params_file}: "
        f"{self.input_vars}"
      )

    self.input_mean = np.asarray(
      params["input_mean"],
      dtype=np.float32,
    ).reshape(-1)

    self.input_std = np.asarray(
      params["input_std"],
      dtype=np.float32,
    ).reshape(-1)

    if self.input_mean.size != self.input_size:
      raise ValueError(
        f"Invalid input_mean size in {params_file}: "
        f"{self.input_mean.size}, expected {self.input_size}"
      )

    if self.input_std.size != self.input_size:
      raise ValueError(
        f"Invalid input_std size in {params_file}: "
        f"{self.input_std.size}, expected {self.input_size}"
      )

    if not np.all(np.isfinite(self.input_mean)):
      raise ValueError(
        f"Invalid input_mean in {params_file}: "
        "all values must be finite"
      )

    if not np.all(np.isfinite(self.input_std)):
      raise ValueError(
        f"Invalid input_std in {params_file}: "
        "all values must be finite"
      )

    if np.any(self.input_std <= 0.0):
      raise ValueError(
        f"Invalid input_std in {params_file}: "
        "all values must be greater than zero"
      )

    self.layers: list[tuple[np.ndarray, np.ndarray, str]] = []
    self.friction_override = False

    for layer_params in params["layers"]:
      weight_key = next(
        key for key in layer_params
        if key.endswith("_W")
      )
      bias_key = next(
        key for key in layer_params
        if key.endswith("_b")
      )

      weights = np.asarray(
        layer_params[weight_key],
        dtype=np.float32,
      ).T

      bias = np.asarray(
        layer_params[bias_key],
        dtype=np.float32,
      ).reshape(-1)

      if zero_bias:
        bias = np.zeros_like(bias)

      activation = str(layer_params["activation"])

      for old_name, new_name in self.activation_function_names.items():
        activation = activation.replace(old_name, new_name)

      self.layers.append((weights, bias, activation))

    self._validate_layers()
    self._check_for_friction_override()

  @staticmethod
  def sigmoid(x: np.ndarray) -> np.ndarray:
    x = np.clip(x, -60.0, 60.0)
    return 1.0 / (1.0 + np.exp(-x))

  @staticmethod
  def identity(x: np.ndarray) -> np.ndarray:
    return x

  @staticmethod
  def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(0.0, x)

  @staticmethod
  def tanh(x: np.ndarray) -> np.ndarray:
    return np.tanh(x)

  def _validate_layers(self) -> None:
    for _, _, activation in self.layers:
      if not hasattr(self, activation):
        raise ValueError(f"Unknown NNFF activation: {activation}")

  def forward(self, x: np.ndarray) -> np.ndarray:
    for weights, bias, activation in self.layers:
      x = getattr(self, activation)(x.dot(weights) + bias)
    return x

  def evaluate(self, input_array: list[float]) -> float:
    values = list(input_array)
    if len(values) != self.input_size:
      if 2 <= len(values) < self.input_size:
        values.extend([0.0] * (self.input_size - len(values)))
      else:
        raise ValueError(f"NNFF input length {len(values)}, expected {self.input_size}")

    x = np.asarray(values, dtype=np.float32)
    x = (x - self.input_mean) / self.input_std
    output = self.forward(x)
    return float(np.asarray(output).reshape(-1)[0])

  def _check_for_friction_override(self) -> None:
    self.friction_override = self.evaluate([10.0, 0.0, 0.2]) < 0.1


class NanoFFModel:
  def __init__(self, weights_loc: str, platform: str):
    self.weights_loc = weights_loc
    self.platform = platform
    self._load_weights(platform)

  def _load_weights(self, platform: str) -> None:
    with open(self.weights_loc, encoding="utf-8") as f:
      self.weights = {key: np.asarray(value) for key, value in json.load(f)[platform].items()}

  @staticmethod
  def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(0.0, x)

  def forward(self, x: np.ndarray) -> np.ndarray:
    if x.ndim != 1:
      raise ValueError("NanoFF input must be one-dimensional")
    x = (x - self.weights["input_norm_mat"][:, 0]) / (
      self.weights["input_norm_mat"][:, 1] - self.weights["input_norm_mat"][:, 0]
    )
    x = self.relu(np.dot(x, self.weights["w_1"]) + self.weights["b_1"])
    x = self.relu(np.dot(x, self.weights["w_2"]) + self.weights["b_2"])
    x = self.relu(np.dot(x, self.weights["w_3"]) + self.weights["b_3"])
    return np.dot(x, self.weights["w_4"]) + self.weights["b_4"]

  def predict(self, x: list[float], do_sample: bool = False) -> float:
    prediction = self.forward(np.asarray(x, dtype=float))
    if do_sample:
      value = np.random.laplace(prediction[0], np.exp(prediction[1]) / self.weights["temperature"])
    else:
      value = prediction[0]
    value = value * (self.weights["output_norm_mat"][1] - self.weights["output_norm_mat"][0]) + self.weights["output_norm_mat"][0]
    return float(value)


@cache
def get_nano_ff_platforms() -> frozenset[str]:
  try:
    with open(NEURAL_PARAMS_PATH, encoding="utf-8") as f:
      return frozenset(json.load(f).keys())
  except (OSError, ValueError, TypeError):
    return frozenset()


def get_nn_model_path(car: str, eps_firmware: str = "", minimum_similarity: float = 0.90) -> str | None:
  if not os.path.isdir(TORQUE_NN_MODEL_PATH):
    return None

  car_name = _normalize_name(car)
  eps_name = _normalize_name(eps_firmware)
  checks = [f"{car_name} {eps_name}"] if len(eps_name) > 3 else []
  checks.append(car_name)

  model_files = [filename for filename in os.listdir(TORQUE_NN_MODEL_PATH) if filename.endswith(".json")]
  for check_name in checks:
    best_path = None
    best_score = -1.0
    for filename in model_files:
      model_name = _normalize_name(os.path.splitext(filename)[0])
      score = _similarity(model_name, check_name)
      if score > best_score:
        best_score = score
        best_path = os.path.join(TORQUE_NN_MODEL_PATH, filename)

    if best_path is not None:
      model_name = _normalize_name(os.path.splitext(os.path.basename(best_path))[0])
      if best_score >= minimum_similarity and car_name in model_name:
        return best_path

  return None


def get_nn_model(car: str, eps_firmware: str = "") -> tuple[FluxModel | None, str | None]:
  model_path = get_nn_model_path(car, eps_firmware)
  if model_path is None:
    return None, None

  try:
    return FluxModel(model_path), model_path
  except (OSError, ValueError, KeyError, TypeError) as exc:
    print(f"NNFF model load failed: {model_path}: {exc}")
    return None, model_path
