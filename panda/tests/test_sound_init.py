"""Source guards for the C4 boot-time register divergence reproduced on hardware."""
from pathlib import Path
import re


SOURCE = (Path(__file__).resolve().parents[1] / 'board/stm32h7/sound.h').read_text()


def test_unused_unclocked_sai1_is_not_written():
  code = re.sub(r'//[^\n]*', '', SOURCE)
  assert 'SAI1->' not in code
  assert 'register_set(&SAI4->GCR,' in code


def test_microphone_shift_is_configured_before_channel_enable():
  statements = re.findall(r'register_set\(&DFSDM1_Channel3->(CHCFGR[12]), ([^;]+);', SOURCE)
  assert [register for register, _ in statements] == ['CHCFGR2', 'CHCFGR1']
  assert 'DFSDM_CHCFGR2_OFFSET_Msk | DFSDM_CHCFGR2_DTRBS_Msk' in statements[0][1]
  assert '2U << DFSDM_CHCFGR2_DTRBS_Pos' in statements[0][1]
  assert 'DFSDM_CHCFGR1_CHEN' in statements[1][1]
