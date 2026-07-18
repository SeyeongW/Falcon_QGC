#!/usr/bin/env bash
# One-time host setup so QGC (VTOL-GCS) can get a coarse "laptop GPS" fix from
# geoclue2 when no vehicle GPS is available.
#
# Why this is needed:
#  * geoclue's default WiFi backend was Mozilla Location Service, shut down in
#    2024, so out-of-the-box WiFi geolocation returns nothing. We point it at
#    BeaconDB (https://beacondb.net), a free, key-less MLS replacement.
#  * geoclue only serves apps it is told to trust. We allowlist QGC's desktop id
#    (org.mavlink.qgroundcontrol) so it doesn't need a session agent.
#
# Run once with root:   sudo bash custom/tools/setup_geoclue.sh
# It is idempotent and backs up the original config.
set -euo pipefail

CONF=/etc/geoclue/geoclue.conf
URL=https://api.beacondb.net/v1/geolocate
APP_IDS=("org.mavlink.qgroundcontrol" "VTOL-GCS")

if [[ $EUID -ne 0 ]]; then
    echo "Please run as root:  sudo bash $0" >&2
    exit 1
fi
if [[ ! -f "$CONF" ]]; then
    echo "geoclue config not found at $CONF (is geoclue-2.0 installed?)" >&2
    exit 1
fi

cp -n "$CONF" "$CONF.bak.$(date +%Y%m%d%H%M%S)" || true

# 1) Point the [wifi] source at BeaconDB (insert a url= line right after the
#    [wifi] header if one is not already active).
if grep -qE '^\s*url\s*=' "$CONF"; then
    sed -i -E "s|^\s*url\s*=.*|url=$URL|" "$CONF"
    echo "updated existing wifi url -> $URL"
else
    awk -v url="$URL" '
        /^\[wifi\]/ { print; print "url=" url; next }
        { print }
    ' "$CONF" > "$CONF.tmp" && mv "$CONF.tmp" "$CONF"
    echo "inserted wifi url -> $URL"
fi

# 2) Allowlist the app id(s) so geoclue grants access without an agent.
for id in "${APP_IDS[@]}"; do
    if grep -qE "^\[$id\]" "$CONF"; then
        echo "allowlist already present: [$id]"
    else
        {
            echo ""
            echo "[$id]"
            echo "allowed=true"
            echo "system=true"
            echo "users="
        } >> "$CONF"
        echo "added allowlist: [$id]"
    fi
done

echo "done. Restart VTOL-GCS; the map should center on your (coarse) location"
echo "when no vehicle is connected."
