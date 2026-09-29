"""Wi-Fi (OTA) firmware update for a specific SlimeVR tracker.

Reproduces what the SlimeVR server does behind its "Update now" button
(server/core/.../firmware/OTAUpdateTask.kt): a UDP invite on port 8266,
authentication with the firmware's public OTA password, and a TCP upload
in 2 KB chunks. Useful when the app refuses to offer the update (e.g. a
firmware reporting version "master", which is not semver).

    python tools/ota_update.py --tracker "Tracker 02D00" --bin firmware/BOARD_SLIMEVR_V1_2-firmware-v0.7.3.bin
    python tools/ota_update.py --tracker "Tracker 02D00" --bin ... --dry-run   # only reports what it would do

Safety: the ESP8266 writes the new image to spare flash and only swaps it
in after validating the MD5. If the upload fails halfway, the old firmware
stays. Worst case, recovery is over USB.
"""

import argparse
import asyncio
import hashlib
import socket
import sys
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import websockets

import slimevr_client as C
import solarxr_fb as fb

OTA_PORT = 8266
OTA_PASSWORD = "SlimeVR-OTA"
FLASH = 0
AUTH = 200
CHUNK = 2048
MIN_BATTERY_PCT = 50


def md5(b):
    return hashlib.md5(b).hexdigest()


async def find_tracker(name):
    """Look the device up by name on the SlimeVR server."""
    async with websockets.connect(C.DEFAULT_URL, max_size=None) as ws:
        await ws.send(C.build_start_data_feed(500))
        deadline = time.time() + 10
        while time.time() < deadline:
            raw = await asyncio.wait_for(ws.recv(), 10)
            if isinstance(raw, str):
                continue
            b = fb.MessageBundle.GetRootAs(raw, 0)
            for i in range(b.DataFeedMsgsLength()):
                h = b.DataFeedMsgs(i)
                if h.MessageType() != fb.DataFeedMessage.DataFeedUpdate:
                    continue
                u = fb.DataFeedUpdate()
                u.Init(h.Message().Bytes, h.Message().Pos)
                for d in range(u.DevicesLength()):
                    dev = u.Devices(d)
                    hi, hs = dev.HardwareInfo(), dev.HardwareStatus()
                    names = {(dev.CustomName() or b"").decode()}
                    for j in range(dev.TrackersLength()):
                        info = dev.Trackers(j).Info()
                        if info:
                            names |= {(info.DisplayName() or b"").decode(), (info.CustomName() or b"").decode()}
                    if name not in names or hi is None or hi.IpAddress() is None:
                        continue
                    ip = socket.inet_ntoa(hi.IpAddress().Addr().to_bytes(4, "big"))
                    return {
                        "ip": ip,
                        "fw": (hi.FirmwareVersion() or b"").decode(),
                        "board": hi.OfficialBoardType(),
                        "battery_pct": hs.BatteryPctEstimate() if hs else None,
                        "battery_v": hs.BatteryVoltage() if hs else None,
                    }
    return None


def authenticate(ip, local_port, fw):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(10)
        s.sendto(f"{FLASH} {local_port} {len(fw)} {md5(fw)}\n".encode(), (ip, OTA_PORT))
        data = s.recv(64).decode(errors="replace")
        if data == "OK":
            return True
        parts = data.split(" ")
        if len(parts) != 2 or parts[0] != "AUTH":
            print(f"unexpected reply to the invite: {data!r}")
            return False
        nonce = parts[1]
        cnonce = md5(uuid.uuid4().hex.encode())
        response = md5(f"{md5(OTA_PASSWORD.encode())}:{nonce}:{cnonce}".encode())
        s.sendto(f"{AUTH} {cnonce} {response}\n".encode(), (ip, OTA_PORT))
        data = s.recv(64).decode(errors="replace")
        if data != "OK":
            print(f"authentication refused: {data!r}")
        return data == "OK"


def upload(server, fw):
    server.settimeout(15)
    conn, addr = server.accept()
    print(f"tracker connected from {addr[0]}, sending {len(fw)} bytes")
    with conn:
        conn.settimeout(5)
        offset, last_pct = 0, -10
        while offset < len(fw):
            chunk = fw[offset:offset + CHUNK]
            conn.sendall(chunk)
            offset += len(chunk)
            ack = b""
            while len(ack) < 4:
                part = conn.recv(4 - len(ack))
                if not part:
                    raise ConnectionError("tracker closed the connection mid-upload")
                ack += part
            pct = offset * 100 // len(fw)
            if pct >= last_pct + 10:
                print(f"  {pct:3d}%", flush=True)
                last_pct = pct
        print("upload complete, waiting for the tracker to validate the image...")
        conn.settimeout(15)
        resp = b""
        try:
            while True:
                part = conn.recv(64)
                if not part:
                    break
                resp += part
        except socket.timeout:
            pass
    return b"OK" in resp


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tracker", required=True, help='tracker name in SlimeVR, e.g. "Tracker 02D00"')
    ap.add_argument("--bin", required=True, type=Path)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")

    fw = args.bin.read_bytes()
    if fw[:1] != b"\xe9":
        sys.exit("file does not look like an ESP firmware image (magic byte != 0xE9)")

    dev = asyncio.run(find_tracker(args.tracker))
    if dev is None:
        sys.exit(f"{args.tracker!r} not found on the SlimeVR server (is it powered and connected?)")
    board = C._enum_names(fb.BoardType).get(dev["board"], dev["board"]) if hasattr(fb, "BoardType") else dev["board"]
    print(f"{args.tracker}: ip={dev['ip']} firmware={dev['fw']} board={board} "
          f"battery={dev['battery_pct']}% ({dev['battery_v']:.2f} V)")
    print(f"image: {args.bin.name} ({len(fw)} bytes, md5 {md5(fw)})")

    if board != "SLIMEVR_V1_2" or "V1_2" not in args.bin.name:
        sys.exit("board and file do not match SlimeVR v1.2, aborting")
    if dev["battery_pct"] is not None and dev["battery_pct"] < MIN_BATTERY_PCT:
        sys.exit(f"battery below {MIN_BATTERY_PCT}%, charge first (the app blocks this too)")
    if args.dry_run:
        print("dry-run: nothing sent")
        return

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind(("0.0.0.0", 0))
        server.listen(1)
        port = server.getsockname()[1]
        print(f"inviting the tracker (local port {port})...")
        if not authenticate(dev["ip"], port, fw):
            sys.exit("authentication failed; nothing was written, the old firmware stays")
        print("authenticated")
        try:
            ok = upload(server, fw)
        except socket.timeout:
            sys.exit("the tracker never connected back (Windows firewall blocking Python?). "
                     "Nothing was written, the old firmware stays.")
    if ok:
        print("OK! The tracker will reboot into the new firmware.")
    else:
        sys.exit("the tracker did not confirm the write. It should still be on the old "
                 "firmware; check that it reconnects.")


if __name__ == "__main__":
    main()
