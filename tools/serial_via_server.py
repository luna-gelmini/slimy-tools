"""Send commands to the serial console of a USB-connected tracker through the
SlimeVR server (the same path the app's "Serial Console" uses). Useful when
the server is already holding the COM port.

    python tools/serial_via_server.py --port COM4 "GET CONFIG" "GET INFO"

Writes the output to firmware/serial_<port>_<time>.txt and closes the port
on the server afterwards (freeing it for esptool).
"""

import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import flatbuffers
import websockets

import slimevr_client as C
import solarxr_fb as fb


def rpc(build):
    """build(b) -> (message_type, offset)."""
    b = flatbuffers.Builder(128)
    mtype, off = build(b)
    fb.RpcMessageHeaderStart(b)
    fb.RpcMessageHeaderAddMessageType(b, mtype)
    fb.RpcMessageHeaderAddMessage(b, off)
    return C._finish_bundle(b, rpc_hdr=fb.RpcMessageHeaderEnd(b))


def open_serial(port):
    def build(b):
        p = b.CreateString(port)
        fb.OpenSerialRequestStart(b)
        fb.OpenSerialRequestAddAuto(b, False)
        fb.OpenSerialRequestAddPort(b, p)
        return fb.RpcMessage.OpenSerialRequest, fb.OpenSerialRequestEnd(b)
    return rpc(build)


def close_serial():
    def build(b):
        fb.CloseSerialRequestStart(b)
        return fb.RpcMessage.CloseSerialRequest, fb.CloseSerialRequestEnd(b)
    return rpc(build)


def custom_command(cmd):
    def build(b):
        s = b.CreateString(cmd)
        fb.SerialTrackerCustomCommandRequestStart(b)
        fb.SerialTrackerCustomCommandRequestAddCommand(b, s)
        return fb.RpcMessage.SerialTrackerCustomCommandRequest, fb.SerialTrackerCustomCommandRequestEnd(b)
    return rpc(build)


async def collect(ws, seconds, sink):
    end = time.time() + seconds
    while time.time() < end:
        try:
            raw = await asyncio.wait_for(ws.recv(), max(0.05, end - time.time()))
        except asyncio.TimeoutError:
            break
        if isinstance(raw, str):
            continue
        bundle = fb.MessageBundle.GetRootAs(raw, 0)
        for i in range(bundle.RpcMsgsLength()):
            h = bundle.RpcMsgs(i)
            if h.MessageType() != fb.RpcMessage.SerialUpdateResponse:
                continue
            r = fb.SerialUpdateResponse()
            r.Init(h.Message().Bytes, h.Message().Pos)
            if r.Log():
                text = r.Log().decode("utf-8", "replace")
                sink.append(text)
                print(text, end="", flush=True)
            if r.Closed():
                sink.append("\n[port closed by the server]\n")


async def main(port, commands, wait):
    out = []
    async with websockets.connect(C.DEFAULT_URL, max_size=None) as ws:
        await ws.send(open_serial(port))
        await collect(ws, 2.0, out)
        for cmd in commands:
            out.append(f"\n>>> {cmd}\n")
            print(f"\n>>> {cmd}", flush=True)
            await ws.send(custom_command(cmd))
            await collect(ws, wait, out)
        await ws.send(close_serial())
        await collect(ws, 1.0, out)
    dest = Path(__file__).resolve().parent.parent / "firmware" / f"serial_{port}_{time.strftime('%Y%m%d-%H%M%S')}.txt"
    dest.write_text("".join(out), encoding="utf-8")
    print(f"\n\nsaved to {dest}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--wait", type=float, default=3.0, help="seconds to wait for each command's output")
    ap.add_argument("commands", nargs="+")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")
    asyncio.run(main(args.port, args.commands, args.wait))
