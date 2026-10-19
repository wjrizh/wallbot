package com.wallbot.controller;

import android.util.Log;

import com.getcapacitor.JSObject;
import com.getcapacitor.Plugin;
import com.getcapacitor.PluginCall;
import com.getcapacitor.PluginMethod;
import com.getcapacitor.annotation.CapacitorPlugin;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

@CapacitorPlugin(name = "Discovery")
public class DiscoveryPlugin extends Plugin {

    private static final String TAG = "WallbotDiscovery";
    private static final int PORT = 48899;
    private static final String REQUEST = "WALLBOT_DISCOVER";
    private static final String PREFIX = "WALLBOT:";
    private static final int TIMEOUT_MS = 1600;

    private volatile JSObject lastResult = null;

    @PluginMethod
    public void discover(PluginCall call) {
        new Thread(() -> {
            JSObject result = scan(TIMEOUT_MS);
            lastResult = result;
            call.resolve(result);
        }, "wallbot-discovery").start();
    }

    @PluginMethod
    public void getLastResult(PluginCall call) {
        call.resolve(lastResult != null ? lastResult : emptyResult());
    }

    private static JSObject emptyResult() {
        JSObject o = new JSObject();
        o.put("host", "");
        o.put("camHost", "");
        o.put("camChild", "");
        return o;
    }

    private JSObject scan(int timeoutMs) {
        String host = "";
        String camHost = "";
        String camChild = "";

        try (DatagramSocket socket = new DatagramSocket()) {
            socket.setBroadcast(true);
            socket.setSoTimeout(200);

            byte[] payload = REQUEST.getBytes("UTF-8");
            for (InetAddress addr : broadcastTargets()) {
                try {
                    socket.send(new DatagramPacket(payload, payload.length, addr, PORT));
                    Log.i(TAG, "discover -> " + addr.getHostAddress());
                } catch (Exception e) {
                    Log.w(TAG, "send failed " + addr.getHostAddress() + ": " + e.getMessage());
                }
            }

            byte[] buf = new byte[128];
            long deadline = System.currentTimeMillis() + timeoutMs;
            while (System.currentTimeMillis() < deadline) {
                DatagramPacket packet = new DatagramPacket(buf, buf.length);
                try {
                    socket.receive(packet);
                } catch (Exception e) {
                    continue;
                }
                String msg = new String(packet.getData(), 0, packet.getLength(), "UTF-8").trim();
                if (!msg.startsWith(PREFIX)) continue;
                String[] parts = msg.split(":");
                if (parts.length < 3) continue;
                String role = parts[1];
                String ip = parts[2];
                Log.i(TAG, "reply " + msg);
                if ("host".equals(role) && host.isEmpty()) host = ip;
                else if ("camhost".equals(role) && camHost.isEmpty()) camHost = ip;
                else if ("camchild".equals(role) && camChild.isEmpty()) camChild = ip;
                if (!host.isEmpty() && !camHost.isEmpty() && !camChild.isEmpty()) break;
            }
        } catch (Exception e) {
            Log.e(TAG, "scan error: " + e.getMessage());
        }

        JSObject o = new JSObject();
        o.put("host", host);
        o.put("camHost", camHost);
        o.put("camChild", camChild);
        return o;
    }

    private static List<InetAddress> broadcastTargets() {
        Set<InetAddress> set = new LinkedHashSet<>();
        try {
            set.add(InetAddress.getByName("255.255.255.255"));
        } catch (Exception ignored) {
        }
        try {
            Enumeration<NetworkInterface> ifaces = NetworkInterface.getNetworkInterfaces();
            while (ifaces.hasMoreElements()) {
                NetworkInterface ni = ifaces.nextElement();
                if (!ni.isUp() || ni.isLoopback()) continue;
                for (InterfaceAddress ia : ni.getInterfaceAddresses()) {
                    InetAddress b = ia.getBroadcast();
                    if (b != null) set.add(b);
                }
            }
        } catch (Exception ignored) {
        }
        return new ArrayList<>(set);
    }
}
