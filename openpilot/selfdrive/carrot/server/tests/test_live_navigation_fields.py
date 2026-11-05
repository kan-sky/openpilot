import ast
from pathlib import Path

from openpilot.cereal import custom
from openpilot.selfdrive.carrot.server.live_runtime.snapshot import _build_carrot


def select_live_services(snapshot):
  path = Path(__file__).resolve().parents[1] / 'features/system.py'
  tree = ast.parse(path.read_text())
  nodes = [node for node in tree.body if (
    isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == '_LIVE_RUNTIME_SERVICE_NAMES' for t in node.targets)
    or isinstance(node, ast.FunctionDef) and node.name == '_select_live_runtime_services')]
  scope = {'Any': object}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), scope)
  return scope['_select_live_runtime_services'](snapshot)


def test_web_runtime_includes_navigation_status_and_traffic():
  msg = custom.CarrotMan.new_message(
    externalNavigationSource='navdy', externalNavigationActive=True,
    routeTrafficTotalMeters=500, routeTrafficSegments=[{'endM': 500, 'status': 'O'}])
  snapshot = {'services': {'carrotMan': _build_carrot(msg), 'navRoute': {},
                           'navInstructionCarrot': {}, 'carState': {'excluded': True}}}
  services = select_live_services(snapshot)
  assert set(services) == {'carrotMan', 'navRoute', 'navInstructionCarrot'}
  assert services['carrotMan']['externalNavigationActive'] is True
  assert services['carrotMan']['routeTrafficSegments'] == [{'endM': 500, 'status': 'O'}]


def test_web_runtime_omits_dead_service():
  assert select_live_services({'services': {'carrotMan': None}}) == {}
  assert select_live_services({}) == {}


def test_actual_web_startup_subscribes_to_exposed_navigation_services():
  from openpilot.selfdrive.carrot.server.live_runtime.services import build_service_list

  path = Path(__file__).resolve().parents[1] / 'app.py'
  tree = ast.parse(path.read_text())
  call = next(node for node in ast.walk(tree) if isinstance(node, ast.Call)
              and isinstance(node.func, ast.Name) and node.func.id == 'RealtimeBroker')
  config = {arg.arg: ast.literal_eval(arg.value) for arg in call.keywords
            if arg.arg in ('include_optional', 'exclude_services')}
  subscribed = build_service_list(include_optional=config['include_optional'], exclude=config['exclude_services'])
  exposed = select_live_services({'services': {name: {} for name in ['carrotMan', 'navInstructionCarrot', 'navRoute']}})
  assert set(exposed) <= set(subscribed)
