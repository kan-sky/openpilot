import numpy as np
import pytest
from types import SimpleNamespace as NS

from openpilot.selfdrive.navigation.helpers import MAX_MANEUVERS, VALHALLA_SIDED_TYPES, navigation_input, valhalla_type


@pytest.mark.parametrize('kind,modifier,command', [
  ('turn', 'straight', 8), ('turn', 'slight right', 9), ('turn', 'right', 10),
  ('turn', 'sharp right', 11), ('turn', 'uturn', 13), ('turn', 'sharp left', 14),
  ('turn', 'left', 15), ('turn', 'slight left', 16), ('on ramp', 'left', 19),
  ('off ramp', 'right', 20), ('fork', 'right', 23), ('rotary', 'left', 26),
  ('depart', '', 1), ('on ramp', 'straight', 17), ('fork', 'slight left', 24),
  ('off ramp', 'sharp right', 20),
  ('exit roundabout', '', 27), ('notification', '', 28),
  ('merge', 'left', None), ('invalid', '', None),
])
def test_valhalla_command_contract(kind, modifier, command):
  assert valhalla_type(kind, modifier) == command


def test_first_supported_nonpassed_maneuver_and_distance_units():
  maneuvers = [
    NS(type='depart', modifier='left', distance=100),
    NS(type='arrive', modifier='left', distance=100),
    NS(type='merge', modifier='left', distance=100),
    NS(type='turn', modifier='right', distance=10),
    NS(type='turn', modifier='left', distance=120),
    NS(type='turn', modifier='right', distance=300),
  ]
  actual = navigation_input(maneuvers, 20.)
  expected = np.zeros(47, np.float32)
  expected[15] = expected[-1] = 1
  np.testing.assert_array_equal(actual, expected)


@pytest.mark.parametrize('count,index', [(33, 0), (33, 32), (2048, 2047)])
def test_full_route_maneuvers_with_bounded_scan(count, index):
  maneuvers = [NS(type='turn', modifier='right', distance=-1.) for _ in range(count)]
  maneuvers[index] = NS(type='turn', modifier='left', distance=120.)
  expected = np.zeros(47, np.float32)
  expected[15], expected[-1] = 1., 1.
  np.testing.assert_array_equal(navigation_input(maneuvers, 20.), expected)


def test_route_over_upstream_limit_rejected_before_reading_maneuvers():
  class OversizedRoute:
    def __len__(self):
      return 2049

    def __iter__(self):
      raise AssertionError('Oversized route must not be scanned')

  assert MAX_MANEUVERS == 2048
  with pytest.raises(ValueError, match='Invalid maneuver list'):
    navigation_input(OversizedRoute(), 0.)


@pytest.mark.parametrize('kind', [*VALHALLA_SIDED_TYPES, 'turn', 'continue', 'new name', 'end of road'])
@pytest.mark.parametrize('modifier', ['not-a-modifier', 'not-right', 'left-ish'])
def test_unknown_modifier_cannot_synthesize_command_or_skip_to_later_turn(kind, modifier):
  assert valhalla_type(kind, modifier) is None
  maneuvers = [NS(type=kind, modifier=modifier, distance=50.), NS(type='turn', modifier='left', distance=120.)]
  with pytest.raises(ValueError, match='Invalid maneuver modifier'):
    navigation_input(maneuvers, 0.)


@pytest.mark.parametrize('kind', ['turn', 'continue', 'new name', 'end of road'])
def test_turn_requires_explicit_modifier(kind):
  assert valhalla_type(kind, '') is None
  with pytest.raises(ValueError, match='Invalid maneuver modifier'):
    navigation_input([NS(type=kind, modifier='', distance=50.)], 0.)


@pytest.mark.parametrize('maneuvers', [
  None, [None], [NS(type=[], modifier='', distance=10)],
  [NS(type='turn', modifier='left', distance=True)],
  [NS(type='turn', modifier='left', distance=10 ** 400)],
  [{}] * (MAX_MANEUVERS + 1),
])
def test_malformed_or_unbounded_tbt_rejected(maneuvers):
  with pytest.raises((ValueError, TypeError, AttributeError)):
    navigation_input(maneuvers, 0.)


@pytest.mark.parametrize('distance', [-1., 1e39])
def test_unusable_distance_has_no_command(distance):
  assert not navigation_input([NS(type='turn', modifier='left', distance=distance)], 0.).any()
