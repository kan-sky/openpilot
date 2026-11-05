package com.hl.navdy.btbench;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;

/** Presents the standard user confirmation; never silently accepts pairing. */
public final class PairActivity extends Activity {
  @Override public void onCreate(Bundle state) {
    super.onCreate(state);
    BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
    if (adapter == null || !adapter.isEnabled()) {
      Log.e("HLBtBench", "Radio must already be enabled");
      finish();
      return;
    }
    Log.i("HLBtBench", "Pair setup: scanMode=" + adapter.getScanMode()
        + " bondedPeers=" + adapter.getBondedDevices().size());
    if (state == null) {
      Intent request = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
      request.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 120);
      startActivityForResult(request, 1);
    }
  }

  @Override protected void onActivityResult(int request, int result, Intent data) {
    super.onActivityResult(request, result, data);
    Log.i("HLBtBench", "Discoverability request result=" + result);
    finish();
  }
}
