"""Distinguish an absent observation from an observed empty navigation state."""


def is_no_observation(response):
  # WirelessOwner.discard skips the main-thread observation and returns exactly
  # these two nulls. Missing fields, status=1, and other null ACKs are NOT this
  # sentinel: they must still pass through the normal per-component validators.
  return (type(response) is dict and response.keys() == {'navigationFeedback', 'modelNavigation'}
          and response['navigationFeedback'] is None and response['modelNavigation'] is None)


def feedback_shape(response):
  """Diagnostic envelope classification, not permission to retain invalid data."""
  if is_no_observation(response):
    return 'noObservation'
  if (isinstance(response, dict) and type(response.get('cameraSpeedKph')) is int
      and all(isinstance(response.get(key), str) for key in ('cameraSource', 'cameraType', 'cameraDistance'))):
    return 'observation'
  # Legacy/partial ACKs remain subject to the existing component validation;
  # they are counted separately, never silently treated as absent observations.
  return 'incomplete'
