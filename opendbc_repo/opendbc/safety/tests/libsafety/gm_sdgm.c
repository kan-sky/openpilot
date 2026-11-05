// Keep CANPacket allocation and forwarding inside C to use the firmware ABI.
#include "safety.c"

// Normally owned by the CAN driver; no hardware driver runs in this harness.
bool safety_tx_buffered_for_fwd = false;

int gm_test_frame(int operation, unsigned int address, unsigned int bus, const unsigned char *data, unsigned int length) {
  if ((bus > 3U) || (length > 8U)) return -2;
  CANPacket_t packet = {0};
  packet.addr = address;
  packet.bus = bus;
  packet.data_len_code = length;
  for (unsigned int i = 0; i < length; i++) packet.data[i] = data[i];
  can_set_checksum(&packet);
  if (operation == 0) return safety_rx_hook(&packet);
  if (operation == 1) return safety_tx_hook(&packet);
  return safety_fwd_hook(&packet);
}
