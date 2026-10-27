#!/bin/bash
# wifi-reconnect: keep wlan0 on preferred SSIDs
set -u
SSID_PRIMARY="${WIFI_SSID_PRIMARY:-ZKXZH}"
PASS_PRIMARY="${WIFI_PASS_PRIMARY:-12345678}"
SSID_FALLBACK="${WIFI_SSID_FALLBACK:-}"
PASS_FALLBACK="${WIFI_PASS_FALLBACK:-}"
IFACE="${WIFI_IFACE:-wlan0}"
INTERVAL="${WIFI_CHECK_SEC:-15}"
LOGTAG="wifi-reconnect"

log() { logger -t "$LOGTAG" "$*"; echo "$(date -Iseconds) $*"; }

has_ip() {
  ip -4 addr show dev "$IFACE" 2>/dev/null | grep -q 'inet '
}

assoc_ok() {
  local st
  st=$(nmcli -t -f GENERAL.STATE device show "$IFACE" 2>/dev/null | head -1)
  echo "$st" | grep -qi 'connected'
}

try_connect() {
  local ssid="$1" pass="$2"
  [ -z "$ssid" ] && return 1
  log "try connect SSID=$ssid"
  rfkill unblock wifi >/dev/null 2>&1 || true
  nmcli radio wifi on >/dev/null 2>&1 || true
  nmcli device set "$IFACE" managed yes >/dev/null 2>&1 || true
  if [ -n "$pass" ]; then
    nmcli device wifi connect "$ssid" password "$pass" ifname "$IFACE" >/dev/null 2>&1 \
      || nmcli connection up "$ssid" >/dev/null 2>&1 \
      || nmcli connection up id "$ssid" >/dev/null 2>&1
  else
    nmcli device wifi connect "$ssid" ifname "$IFACE" >/dev/null 2>&1 \
      || nmcli connection up "$ssid" >/dev/null 2>&1
  fi
}

reconnect() {
  log "wifi down -> reconnect"
  nmcli device disconnect "$IFACE" >/dev/null 2>&1 || true
  sleep 1
  try_connect "$SSID_PRIMARY" "$PASS_PRIMARY" || true
  sleep 3
  if ! has_ip && [ -n "$SSID_FALLBACK" ]; then
    try_connect "$SSID_FALLBACK" "$PASS_FALLBACK" || true
    sleep 3
  fi
  if has_ip; then
    log "wifi recovered: $(ip -4 -o addr show dev "$IFACE" | awk '{print $4}')"
  else
    log "wifi still down"
  fi
}

log "start watchdog iface=$IFACE primary=$SSID_PRIMARY interval=${INTERVAL}s"
while true; do
  if has_ip && assoc_ok; then
    :
  else
    reconnect
  fi
  sleep "$INTERVAL"
done
