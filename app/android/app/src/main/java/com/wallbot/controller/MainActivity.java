package com.wallbot.controller;

import android.content.Context;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.os.Bundle;
import android.util.Log;
import com.getcapacitor.BridgeActivity;

public class MainActivity extends BridgeActivity {

    private ConnectivityManager cm;
    private ConnectivityManager.NetworkCallback callback;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        bindToWifi();
    }

    /* 母机热点无外网, 系统会把 WiFi 判定为"无网络"并切回蜂窝.
       绑定进程网络到 WiFi, 保证 192.168.4.1 始终可达. */
    private void bindToWifi() {
        cm = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);

        NetworkRequest request = new NetworkRequest.Builder()
                .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                .build();

        callback = new ConnectivityManager.NetworkCallback() {
            @Override
            public void onAvailable(Network network) {
                cm.bindProcessToNetwork(network);
                Log.i("Wallbot", "已绑定 WiFi: " + network);
            }

            @Override
            public void onLost(Network network) {
                cm.bindProcessToNetwork(null);
                Log.i("Wallbot", "WiFi 断开");
            }
        };

        cm.requestNetwork(request, callback);
    }

    @Override
    public void onDestroy() {
        if (cm != null && callback != null) {
            cm.unregisterNetworkCallback(callback);
        }
        super.onDestroy();
    }
}