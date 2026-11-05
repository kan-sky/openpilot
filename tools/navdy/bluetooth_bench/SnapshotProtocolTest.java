package com.hl.navdy.btbench;

import java.io.*;
import java.util.Arrays;
import java.util.zip.Deflater;

/** Standalone protocol checks; no Android or radio dependency. */
public final class SnapshotProtocolTest {
  static final class Clock implements SnapshotProtocol.Clock {
    long now = 1000000000L;
    public long nanos() { return now; }
  }
  static void require(boolean result) { if (!result) throw new AssertionError(); }
  static void rejects(byte[] wire, int size, int flag, int crc) throws Exception {
    try { SnapshotProtocol.decode(wire, size, flag, crc); }
    catch (IOException expected) { return; }
    throw new AssertionError("Invalid frame accepted");
  }
  static byte[] compress(byte[] raw) {
    Deflater d = new Deflater(1);
    d.setInput(raw); d.finish();
    byte[] buf = new byte[40000];
    int n = d.deflate(buf); d.end();
    return Arrays.copyOf(buf, n);
  }
  static long[] exchange(SnapshotProtocol.Session s, int seq, int lease, long delta, byte[] raw, boolean zip) throws Exception {
    byte[] wire = zip ? compress(raw) : raw;
    ByteArrayOutputStream request = new ByteArrayOutputStream();
    DataOutputStream out = new DataOutputStream(request);
    out.writeInt(SnapshotProtocol.REQUEST); out.writeInt(seq); out.writeInt(lease); out.writeLong(delta);
    out.writeInt(raw.length); out.writeInt(wire.length); out.writeInt(Protocol.checksum(raw)); out.writeInt(zip ? 1 : 0);
    out.write(wire);
    ByteArrayOutputStream reply = new ByteArrayOutputStream();
    s.exchange(new DataInputStream(new ByteArrayInputStream(request.toByteArray())), new DataOutputStream(reply));
    DataInputStream in = new DataInputStream(new ByteArrayInputStream(reply.toByteArray()));
    require(in.readInt() == SnapshotProtocol.RESPONSE); require(in.readInt() == seq);
    int status = in.readInt(), next = in.readInt();
    long age = in.readLong(); in.readLong();
    require(in.readInt() == wire.length); int clears = in.readInt();
    byte[] echo = new byte[wire.length]; in.readFully(echo);
    require(Arrays.equals(SnapshotProtocol.decode(echo, raw.length, zip ? 1 : 0, Protocol.checksum(raw)), raw));
    return new long[]{status, next, age, clears};
  }
  public static void main(String[] args) throws Exception {
    byte[] raw = "{\"text\":\"\ud55c\uae00\",\"seq\":42}".getBytes("UTF-8");
    byte[] zipped = compress(raw);
    require(Arrays.equals(raw, SnapshotProtocol.decode(zipped, raw.length, 1, Protocol.checksum(raw))));
    rejects(zipped, 1, 1, 0);
    rejects(zipped, raw.length, 1, 1);
    rejects(Arrays.copyOf(zipped, zipped.length - 1), raw.length, 1, 0);
    rejects(Arrays.copyOf(zipped, zipped.length + 1), raw.length, 1, 0);
    rejects(raw, 32769, 0, 0);
    rejects(raw, raw.length, 2, 0);
    Clock clock = new Clock();
    SnapshotProtocol.Session session = new SnapshotProtocol.Session(clock);
    session.hello(new DataOutputStream(new ByteArrayOutputStream()));
    clock.now += 100000000;
    require(exchange(session, 0, 0, 50000, raw, true)[0] == 0);
    require(session.hasState());
    clock.now += 100000000;
    require(exchange(session, 5, 1, -1000000, raw, true)[0] == 1);
    require(session.hasState());
    // A rejected stale packet must not renew the previous state's lifetime.
    clock.now += 350000000;
    require(!session.hasState()); require(session.expiredClears() == 1);
    require(exchange(session, 6, 2, 300000, raw, false)[0] == 0);
    require(exchange(session, 4, 3, 0, raw, false)[0] == 2);
    require(exchange(session, 7, 4, 1000000, raw, false)[0] == 1);
    require(session.hasState());
    clock.now += 500000000;
    require(!session.hasState()); require(session.expiredClears() == 2);
    boolean invalidLease = false;
    try { exchange(session, 8, 100, 0, raw, true); }
    catch (IOException expected) { invalidLease = true; }
    require(invalidLease);
    System.out.println("Snapshot protocol tests passed (not hardware)");
  }
}
