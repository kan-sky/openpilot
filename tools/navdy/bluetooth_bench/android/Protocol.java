package com.hl.navdy.btbench;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.zip.CRC32;

/** Benchmark-only binary framing; cannot forward vehicle or navigation data. */
public final class Protocol {
  public static final int REQUEST = 0x484c4231;
  public static final int RESPONSE = 0x484c5231;
  public static final int HEADER = 20;
  public static final int MAX_BYTES = 32768;

  public static byte[] pattern(int sequence, int size) {
    byte[] result = new byte[size];
    int value = sequence;
    for (int i = 0; i < size; i++) {
      value = value * 1664525 + 1013904223;
      result[i] = (byte) (value >>> 24);
    }
    return result;
  }

  public static int checksum(byte[] data) {
    CRC32 crc = new CRC32();
    crc.update(data);
    return (int) crc.getValue();
  }

  public static void size(int size) throws IOException {
    if (size < 0 || size > MAX_BYTES - HEADER) throw new IOException("Invalid frame size");
  }

  public static long exchange(DataInputStream in, DataOutputStream out, int previous) throws IOException {
    if (in.readInt() != REQUEST) throw new IOException("Wrong protocol");
    int sequence = in.readInt();
    int tx = in.readInt();
    int rx = in.readInt();
    int crc = in.readInt();
    size(tx);
    size(rx);
    if (sequence < 0 || sequence != previous + 1) throw new IOException("Sequence mismatch");
    byte[] request = new byte[tx];
    in.readFully(request);
    if (checksum(request) != crc || !Arrays.equals(request, pattern(sequence, tx))) {
      throw new IOException("Corrupt request");
    }
    byte[] response = pattern(sequence ^ 0x5a5a5a5a, rx);
    out.writeInt(RESPONSE);
    out.writeInt(sequence);
    out.writeInt(rx);
    out.writeInt(tx);
    out.writeInt(checksum(response));
    out.write(response);
    out.flush();
    return tx + rx + 2L * HEADER;
  }
}
