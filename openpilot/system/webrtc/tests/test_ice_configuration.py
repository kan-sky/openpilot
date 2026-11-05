import asyncio
from unittest.mock import patch

from aiortc import RTCConfiguration, RTCPeerConnection
import pytest

from teleoprtc import WebRTCAnswerBuilder
from openpilot.system.webrtc import webrtcd


@pytest.mark.parametrize("host_only", [False, True])
def test_answer_builder_preserves_optional_ice_configuration(host_only):
  async def check():
    configuration = RTCConfiguration(iceServers=[]) if host_only else None
    with patch("teleoprtc.stream.aiortc.RTCPeerConnection", wraps=RTCPeerConnection) as factory:
      stream = WebRTCAnswerBuilder("v=0\r\n", configuration=configuration).stream()
    try:
      factory.assert_called_once_with(configuration=configuration)
      stream.peer_connection.createDataChannel("test")
      ice = stream.peer_connection.sctp.transport.transport.iceGatherer._connection
      assert (ice.stun_server is None) == host_only
    finally:
      await stream.stop()
  asyncio.run(check())


@pytest.mark.parametrize("carrot_mode", [False, True])
def test_only_carrot_vision_disables_external_stun(monkeypatch, carrot_mode):
  monkeypatch.setattr(webrtcd, "_carrot_vision_mode", carrot_mode)

  async def check():
    offer_peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
    session = None
    try:
      offer_peer.createDataChannel("data")
      offer = await offer_peer.createOffer()
      with patch("teleoprtc.WebRTCAnswerBuilder", wraps=WebRTCAnswerBuilder) as builder:
        session = webrtcd.StreamSession(offer.sdp, [], [], [], debug_mode=True)
      configuration = builder.call_args.kwargs["configuration"]
      if carrot_mode:
        assert configuration.iceServers == []
      else:
        assert configuration is None
    finally:
      if session:
        await session.stream.stop()
      await offer_peer.close()
  asyncio.run(check())


def test_host_only_answer_connects_without_stun():
  async def check():
    config = RTCConfiguration(iceServers=[])
    peer = RTCPeerConnection(config)
    answer_stream = None
    received = asyncio.get_running_loop().create_future()
    try:
      channel = peer.createDataChannel("data")
      offer = await peer.createOffer()
      await peer.setLocalDescription(offer)
      answer_stream = WebRTCAnswerBuilder(peer.localDescription.sdp, configuration=config).stream()
      answer_stream.set_message_handler(lambda data: received.set_result(data))
      answer = await answer_stream.start()
      assert " typ host" in answer.sdp
      assert " typ srflx" not in answer.sdp
      assert " typ relay" not in answer.sdp
      await peer.setRemoteDescription(answer)
      await asyncio.wait_for(answer_stream.wait_for_connection(), 5)
      for _ in range(100):
        if channel.readyState == "open":
          break
        await asyncio.sleep(0.01)
      assert channel.readyState == "open"
      channel.send(b"tailscale-ice-check")
      assert await asyncio.wait_for(received, 2) == b"tailscale-ice-check"
    finally:
      if answer_stream:
        await answer_stream.stop()
      await peer.close()
  asyncio.run(check())
