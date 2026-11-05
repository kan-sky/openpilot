"""Validated pinned-host feedback buffers for serial TensorRT execution."""


def feedback_sources(inputs, outputs, pairs):
  sources = {}
  if len(set(pairs.values())) != len(pairs):
    raise ValueError('Duplicate state feedback output')
  for name, output in pairs.items():
    if name not in inputs or output not in outputs:
      raise ValueError(f'Missing state feedback binding: {name}')
    target, source = inputs[name], outputs[output]
    if target.shape != source.shape or target.dtype != source.dtype or target.nbytes != source.nbytes:
      raise ValueError(f'State feedback contract mismatch: {name}')
    sources[name] = source
  return sources
