package com.hl.navdy.btbench;

import android.app.Service;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.IBinder;
import android.os.PowerManager;
import android.os.SystemClock;
import android.util.Log;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicBoolean;

/** Explicitly started, paired-peer-only trial, independent of the HUD app. */
public final class BenchService extends Service {
  private static final String TAG = "HLBtBench";
  private static final UUID UUID_VALUE = UUID.fromString("b910e2b5-1b15-4b3d-88a0-4d803ed0f7ce");
  private final AtomicBoolean running = new AtomicBoolean(false);
  private volatile BluetoothServerSocket server;
  private volatile BluetoothSocket client;
  private volatile long deadline;
  private volatile long progress;
  private PowerManager.WakeLock wake;

  @Override public IBinder onBind(Intent intent) { return null; }

  @Override public int onStartCommand(Intent intent, int flags, int startId) {
    if (running.get()) return START_NOT_STICKY;
    final String peer = intent == null ? null : intent.getStringExtra("peer");
    BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
    if (peer == null || !BluetoothAdapter.checkBluetoothAddress(peer) || adapter == null
        || !adapter.isEnabled() || adapter.getRemoteDevice(peer).getBondState() != BluetoothDevice.BOND_BONDED) {
      Log.e(TAG, "Refused: exact peer must already be paired and radio enabled");
      stopSelf();
      return START_NOT_STICKY;
    }
    running.set(true);
    deadline = SystemClock.elapsedRealtime() + 600000;
    progress = SystemClock.elapsedRealtime();
    wake = ((PowerManager) getSystemService(POWER_SERVICE)).newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, TAG);
    wake.acquire(610000);
    new Thread(new Runnable() {
      @Override public void run() { serve(peer); }
    }, "hl-bt-bench").start();
    new Thread(new Runnable() {
      @Override public void run() {
        while (running.get()) {
          long now = SystemClock.elapsedRealtime();
          if (now >= deadline || (client != null && now - progress > 10000)) {
            Log.i(TAG, "Closing: trial deadline or stalled frame");
            closeSockets();
            stopSelf();
            return;
          }
          SystemClock.sleep(100);
        }
      }
    }, "hl-bt-watchdog").start();
    return START_NOT_STICKY;
  }

  private void serve(String peer) {
    try {
      server = BluetoothAdapter.getDefaultAdapter().listenUsingRfcommWithServiceRecord("HL Navdy Bench", UUID_VALUE);
      if (!running.get()) { closeSockets(); return; }
      Log.i(TAG, "Listening: secure RFCOMM, expires in ten minutes");
      while (running.get()) {
        BluetoothSocket socket = server.accept();
        if (!running.get()) { socket.close(); break; }
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
        Log.i(TAG, "Client connected");
        try {
          DataInputStream in = new DataInputStream(new BufferedInputStream(socket.getInputStream(), 32768));
          DataOutputStream out = new DataOutputStream(new BufferedOutputStream(socket.getOutputStream(), 32768));
          while (running.get()) {
            bytes += Protocol.exchange(in, out, frames - 1);
            frames++;
            progress = SystemClock.elapsedRealtime();
          }
        } catch (IOException e) {
          Log.i(TAG, "Session ended: " + e.getClass().getSimpleName() + ": " + e.getMessage());
        } finally {
          socket.close();
          client = null;
          Log.i(TAG, "Summary frames=" + frames + " bytes=" + bytes
              + " elapsedMs=" + (SystemClock.elapsedRealtime() - began));
        }
      }
    } catch (IOException e) {
      Log.e(TAG, "Listener ended: " + e.getMessage());
    } finally {
      closeSockets();
      stopSelf();
    }
  }

  private void closeSockets() {
    running.set(false);
    try { if (client != null) client.close(); } catch (IOException ignored) { }
    try { if (server != null) server.close(); } catch (IOException ignored) { }
  }

  @Override public void onDestroy() {
    closeSockets();
    if (wake != null && wake.isHeld()) wake.release();
    super.onDestroy();
  }
}
