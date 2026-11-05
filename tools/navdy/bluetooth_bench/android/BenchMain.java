package com.hl.navdy.btbench;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.os.Looper;
import android.os.SystemClock;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.UUID;

/** Temporary app_process entrypoint for a user-owned, already-root ADB session. */
public final class BenchMain {
  private static volatile BluetoothServerSocket server;
  private static volatile BluetoothSocket client;
  private static volatile boolean running = true;
  private static volatile long progress;
  private static volatile SnapshotProtocol.Session snapshot;

  private static void close() {
    running = false;
    try { if (client != null) client.close(); } catch (IOException ignored) { }
    try { if (server != null) server.close(); } catch (IOException ignored) { }
  }

  public static void main(String[] args) throws Exception {
    Looper.prepareMainLooper();
    BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
    if (adapter == null) throw new IOException("No Bluetooth adapter");
    System.out.println("enabled=" + adapter.isEnabled() + " address=" + adapter.getAddress()
        + " scanMode=" + adapter.getScanMode() + " bondedPeers=" + adapter.getBondedDevices().size());
    if (args.length == 2 && "trial-connectable".equals(args[0])) {
      int mode = adapter.getScanMode();
      if (!BluetoothAdapter.checkBluetoothAddress(args[1]) || !adapter.isEnabled()
          || adapter.getRemoteDevice(args[1]).getBondState() != BluetoothDevice.BOND_BONDED
          || (mode != BluetoothAdapter.SCAN_MODE_NONE && mode != BluetoothAdapter.SCAN_MODE_CONNECTABLE)) {
        throw new IOException("Known paired peer and idle scan policy required");
      }
      if (mode == BluetoothAdapter.SCAN_MODE_NONE) {
        Object result = adapter.getClass().getMethod("setScanMode", int.class, int.class)
            .invoke(adapter, BluetoothAdapter.SCAN_MODE_CONNECTABLE, 0);
        if (!Boolean.TRUE.equals(result)) throw new IOException("Connectable trial refused");
      }
      SystemClock.sleep(300);
      System.out.println("trialScanMode=" + adapter.getScanMode());
      System.exit(adapter.getScanMode() == BluetoothAdapter.SCAN_MODE_CONNECTABLE ? 0 : 1);
    }
    if (args.length == 1 && "restore-original-scan-none".equals(args[0])) {
      int mode = adapter.getScanMode();
      if (mode != BluetoothAdapter.SCAN_MODE_NONE && mode != BluetoothAdapter.SCAN_MODE_CONNECTABLE) {
        throw new IOException("New discoverability request present; do not overwrite it");
      }
      if (mode == BluetoothAdapter.SCAN_MODE_CONNECTABLE) {
        Object result = adapter.getClass().getMethod("setScanMode", int.class, int.class)
            .invoke(adapter, BluetoothAdapter.SCAN_MODE_NONE, 0);
        if (!Boolean.TRUE.equals(result)) throw new IOException("Scan-mode restoration refused");
      }
      SystemClock.sleep(300);
      System.out.println("restoredScanMode=" + adapter.getScanMode());
      System.exit(adapter.getScanMode() == BluetoothAdapter.SCAN_MODE_NONE ? 0 : 1);
    }
    if (args.length == 1 && "status".equals(args[0])) {
      for (BluetoothDevice device : adapter.getBondedDevices()) {
        Object connected = "unknown";
        try { connected = device.getClass().getMethod("isConnected").invoke(device); }
        catch (ReflectiveOperationException ignored) { }
        System.out.println("bonded name=" + device.getName() + " connected=" + connected);
      }
      System.exit(0);
    }
    if (args.length != 2 || !("serve".equals(args[0]) || "serve-snapshot".equals(args[0]))
        || !BluetoothAdapter.checkBluetoothAddress(args[1])) {
      throw new IllegalArgumentException("Use status, serve or serve-snapshot EXACT_PAIRED_COMMA_MAC");
    }
    final boolean snapshots = "serve-snapshot".equals(args[0]);
    final String peer = args[1];
    if (!adapter.isEnabled() || adapter.getRemoteDevice(peer).getBondState() != BluetoothDevice.BOND_BONDED) {
      throw new IOException("Radio enabled and explicitly paired peer required");
    }
    Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
      @Override public void run() { close(); }
    }));
    server = adapter.listenUsingRfcommWithServiceRecord(snapshots ? "HL Snapshot Bench" : "HL Navdy Bench",
        UUID.fromString(snapshots ? "a8de4aa2-5600-47cb-8b2d-3a7dabd3a918" : "b910e2b5-1b15-4b3d-88a0-4d803ed0f7ce"));
    final long deadline = SystemClock.elapsedRealtime() + 600000;
    Thread watcher = new Thread(new Runnable() {
      @Override public void run() {
        while (running) {
          long now = SystemClock.elapsedRealtime();
          SnapshotProtocol.Session active = snapshot;
          if (active != null) active.expire();
          if (now >= deadline || (client != null && now - progress > 10000)) {
            System.out.println("END deadline_or_stall");
            close();
            return;
          }
          SystemClock.sleep(100);
        }
      }
    });
    watcher.setDaemon(true);
    watcher.start();
    System.out.println("READY secure_rfcomm expires_seconds=600");
    try {
      while (running) {
        BluetoothSocket socket = server.accept();
        if (!running) { socket.close(); break; }
        if (!peer.equals(socket.getRemoteDevice().getAddress())
            || socket.getRemoteDevice().getBondState() != BluetoothDevice.BOND_BONDED) {
          socket.close();
          continue;
        }
        progress = SystemClock.elapsedRealtime();
        client = socket;
        int frames = 0;
        long bytes = 0;
        long began = SystemClock.elapsedRealtime();
        System.out.println("CONNECTED");
        try {
          DataInputStream in = new DataInputStream(new BufferedInputStream(socket.getInputStream(), 32768));
          DataOutputStream out = new DataOutputStream(new BufferedOutputStream(socket.getOutputStream(), 32768));
          SnapshotProtocol.Session state = snapshots ? new SnapshotProtocol.Session(new SnapshotProtocol.Clock() {
            @Override public long nanos() { return System.nanoTime(); }
          }) : null;
          snapshot = state;
          if (state != null) state.hello(out);
          while (running) {
            bytes += state == null ? Protocol.exchange(in, out, frames - 1) : state.exchange(in, out);
            frames++;
            progress = SystemClock.elapsedRealtime();
          }
        } catch (IOException e) {
          System.out.println("SESSION_END " + e.getClass().getSimpleName() + ": " + e.getMessage());
        } finally {
          SnapshotProtocol.Session state = snapshot;
          if (state != null) System.out.println("SNAPSHOT expiredClears=" + state.expiredClears());
          snapshot = null;
          socket.close();
          client = null;
          System.out.println("SUMMARY frames=" + frames + " bytes=" + bytes
              + " elapsedMs=" + (SystemClock.elapsedRealtime() - began));
        }
      }
    } finally {
      close();
    }
    System.exit(0);
  }
}
