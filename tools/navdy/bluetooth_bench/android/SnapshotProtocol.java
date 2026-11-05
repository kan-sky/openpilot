package com.hl.navdy.btbench;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.zip.DataFormatException;
import java.util.zip.Inflater;

/** Isolated snapshot sink: never forwards data to the HUD or vehicle. */
public final class SnapshotProtocol {
  public static final int HELLO = 0x484c5330;
  public static final int REQUEST = 0x484c5332;
  public static final int RESPONSE = 0x484c4132;
  public static final int TTL_US = 500000;
  public static final int MAX_BYTES = 32768;
  public interface Clock { long nanos(); }

  public static byte[] decode(byte[] wire, int rawSize, int flags, int crc) throws IOException {
    if (rawSize < 1 || rawSize > MAX_BYTES || wire.length < 1 || wire.length > MAX_BYTES) {
      throw new IOException("Snapshot size out of bounds");
    }
    byte[] raw;
    if (flags == 0) {
      if (wire.length != rawSize) throw new IOException("Raw size mismatch");
      raw = wire;
    } else if (flags == 1) {
      Inflater inflater = new Inflater();
      try {
        inflater.setInput(wire);
        byte[] bounded = new byte[rawSize + 1];
        int count = 0;
        while (!inflater.finished() && count < bounded.length) {
          int n = inflater.inflate(bounded, count, bounded.length - count);
          if (n == 0 && !inflater.finished()) throw new IOException("Incomplete compressed snapshot");
          count += n;
        }
        if (!inflater.finished() || count != rawSize || inflater.getRemaining() != 0) {
          throw new IOException("Compressed size or trailing data mismatch");
        }
        raw = new byte[rawSize];
        System.arraycopy(bounded, 0, raw, 0, rawSize);
      } catch (DataFormatException e) {
        throw new IOException("Invalid compressed snapshot", e);
      } finally {
        inflater.end();
      }
    } else {
      throw new IOException("Unknown encoding");
    }
    if (Protocol.checksum(raw) != crc) throw new IOException("Snapshot checksum mismatch");
    return raw;
  }

  public static final class Session {
    private final Clock clock;
    private long leaseIssued;
    private int lease;
    private int lastSequence = -1;
    private byte[] current;
    private long validUntil;
    private int expiredClears;

    public Session(Clock clock) { this.clock = clock; }

    public void hello(DataOutputStream out) throws IOException {
      leaseIssued = clock.nanos();
      out.writeInt(HELLO);
      out.writeInt(lease);
      out.writeInt(TTL_US);
      out.writeInt(MAX_BYTES);
      out.flush();
    }

    public synchronized void expire() {
      if (current != null && clock.nanos() >= validUntil) {
        current = null;
        expiredClears++;
      }
    }

    public synchronized boolean hasState() { expire(); return current != null; }
    public synchronized int expiredClears() { expire(); return expiredClears; }

    private synchronized int apply(int sequence, byte[] raw, long ageUs, long now) {
      expire();
      if (sequence <= lastSequence) return 2;
      lastSequence = sequence;
      if (ageUs < 0 || ageUs >= TTL_US) return 1;
      current = raw;
      validUntil = now + (TTL_US - ageUs) * 1000L;
      return 0;
    }

    public long exchange(DataInputStream in, DataOutputStream out) throws IOException {
      if (in.readInt() != REQUEST) throw new IOException("Wrong snapshot protocol");
      int sequence = in.readInt();
      int suppliedLease = in.readInt();
      long captureDeltaUs = in.readLong();
      int rawSize = in.readInt();
      int wireSize = in.readInt();
      int crc = in.readInt();
      int flags = in.readInt();
      if (sequence < 0 || suppliedLease != lease || Math.abs(captureDeltaUs) > 10000000L
          || captureDeltaUs == Long.MIN_VALUE || wireSize < 1 || wireSize > MAX_BYTES) {
        throw new IOException("Invalid snapshot sequence, lease or size");
      }
      byte[] wire = new byte[wireSize];
      in.readFully(wire);
      long decodeStart = clock.nanos();
      byte[] raw = decode(wire, rawSize, flags, crc);
      long now = clock.nanos();
      long elapsedUs = (now - leaseIssued) / 1000;
      // Receiver-issued lease avoids wall-clock/absolute-monotonic synchronization.
      // The upper age includes the prior ACK transit, plus 2 ms + 1000 ppm margin.
      long ageUs = elapsedUs - captureDeltaUs + 2000 + Math.max(0, elapsedUs / 1000);
      int status = apply(sequence, raw, ageUs, now);
      lease++;
      leaseIssued = clock.nanos();
      out.writeInt(RESPONSE);
      out.writeInt(sequence);
      out.writeInt(status);
      out.writeInt(lease);
      out.writeLong(ageUs);
      out.writeLong((now - decodeStart) / 1000);
      out.writeInt(wireSize);
      out.writeInt(expiredClears());
      out.write(wire);
      out.flush();
      return 36L + 40L + 2L * wireSize;
    }
  }
}
