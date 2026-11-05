package com.hl.navdy.btbench;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.Arrays;

public final class ProtocolTest {
  private static byte[] request(int seq, int tx, int rx, boolean corrupt) throws IOException {
    ByteArrayOutputStream buf = new ByteArrayOutputStream();
    DataOutputStream out = new DataOutputStream(buf);
    byte[] body = Protocol.pattern(seq, tx);
    out.writeInt(Protocol.REQUEST);
    out.writeInt(seq);
    out.writeInt(tx);
    out.writeInt(rx);
    out.writeInt(Protocol.checksum(body) ^ (corrupt ? 1 : 0));
    out.write(body);
    return buf.toByteArray();
  }

  public static void main(String[] args) throws Exception {
    byte[] known = {(byte) 0x3c, (byte) 0x47, (byte) 0xd1, (byte) 0xaa,
                    (byte) 0x62, (byte) 0x9f, (byte) 0x57, (byte) 0xa3};
    if (!Arrays.equals(known, Protocol.pattern(0, 8))) throw new AssertionError("pattern");
    for (int[] sizes : new int[][] {{0, 0}, {2389, 1912}, {32748, 32748}}) {
      ByteArrayOutputStream buf = new ByteArrayOutputStream();
      long bytes = Protocol.exchange(new DataInputStream(new ByteArrayInputStream(request(0, sizes[0], sizes[1], false))),
          new DataOutputStream(buf), -1);
      if (bytes != sizes[0] + sizes[1] + 40) throw new AssertionError("bytes");
      DataInputStream reply = new DataInputStream(new ByteArrayInputStream(buf.toByteArray()));
      if (reply.readInt() != Protocol.RESPONSE || reply.readInt() != 0
          || reply.readInt() != sizes[1] || reply.readInt() != sizes[0]) throw new AssertionError("header");
      byte[] expected = Protocol.pattern(0x5a5a5a5a, sizes[1]);
      if (reply.readInt() != Protocol.checksum(expected)) throw new AssertionError("crc");
      byte[] actual = new byte[sizes[1]];
      reply.readFully(actual);
      if (!Arrays.equals(actual, expected)) throw new AssertionError("body");
    }
    for (byte[] invalid : new byte[][] {request(1, 2, 3, false), request(0, 2, 3, true),
                                       request(0, 32749, 0, false), new byte[20]}) {
      try {
        Protocol.exchange(new DataInputStream(new ByteArrayInputStream(invalid)),
            new DataOutputStream(new ByteArrayOutputStream()), -1);
        throw new AssertionError("invalid input accepted");
      } catch (IOException expected) { }
    }
    System.out.println("Protocol tests passed (synthetic, no Bluetooth hardware)");
  }
}
