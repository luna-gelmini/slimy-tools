"""SolarXR client for the local SlimeVR server.

The server speaks SolarXR on ws://localhost:21110: binary FlatBuffers
messages, each one a `MessageBundle` holding data feed and RPC messages.
The Python bindings live in solarxr_fb.py (generated, see NOTICE.md).

Usage:
    client = SlimeVRClient()
    async for msg in client.messages():   # reconnects on its own
        if isinstance(msg, FeedUpdate): ...
        elif isinstance(msg, ResetEvent): ...
    await client.reset_yaw(["LEFT_UPPER_LEG"])
"""

import asyncio
import math
import time
from dataclasses import dataclass

import flatbuffers
import websockets

import solarxr_fb as fb

DEFAULT_URL = "ws://localhost:21110"


def _enum_names(enum_cls):
    return {v: k for k, v in vars(enum_cls).items() if not k.startswith("_")}


BODY_PART_NAMES = _enum_names(fb.BodyPart)
BODY_PART_IDS = {v: k for k, v in BODY_PART_NAMES.items()}
IMU_NAMES = _enum_names(fb.ImuType)
STATUS_NAMES = _enum_names(fb.TrackerStatus)
RESET_TYPE_NAMES = _enum_names(fb.ResetType)
RESET_STATUS_NAMES = _enum_names(fb.ResetStatus)



@dataclass
class TrackerSample:
    key: str
    body_part: str
    name: str
    imu: str
    is_imu: bool
    status: str
    quat: tuple | None
    quat_adj: tuple | None
    lin_acc: tuple | None
    temp: float | None
    tps: int | None
    sa_corr: float | None
    sa_locked: bool | None
    sa_locked_err: float | None
    sa_center_err: float | None
    sa_neighbor_err: float | None
    battery_v: float | None = None
    battery_pct: int | None = None
    rssi: int | None = None

    @property
    def lin_acc_mag(self):
        if self.lin_acc is None:
            return None
        return math.sqrt(sum(c * c for c in self.lin_acc))


@dataclass
class FeedUpdate:
    t: float
    trackers: list


@dataclass
class ResetEvent:
    t: float
    reset_type: str
    status: str
    body_parts: list
    progress: int
    duration: int


@dataclass
class ConnectionEvent:
    t: float
    connected: bool
    detail: str = ""



def _finish_bundle(b, data_feed_hdr=None, rpc_hdr=None):
    feed_vec = rpc_vec = None
    if data_feed_hdr is not None:
        fb.MessageBundleStartDataFeedMsgsVector(b, 1)
        b.PrependUOffsetTRelative(data_feed_hdr)
        feed_vec = b.EndVector()
    if rpc_hdr is not None:
        fb.MessageBundleStartRpcMsgsVector(b, 1)
        b.PrependUOffsetTRelative(rpc_hdr)
        rpc_vec = b.EndVector()
    fb.MessageBundleStart(b)
    if feed_vec is not None:
        fb.MessageBundleAddDataFeedMsgs(b, feed_vec)
    if rpc_vec is not None:
        fb.MessageBundleAddRpcMsgs(b, rpc_vec)
    b.Finish(fb.MessageBundleEnd(b))
    return bytes(b.Output())


def build_start_data_feed(interval_ms):
    b = flatbuffers.Builder(256)

    fb.TrackerDataMaskStart(b)
    for add in (
        fb.TrackerDataMaskAddInfo,
        fb.TrackerDataMaskAddStatus,
        fb.TrackerDataMaskAddRotation,
        fb.TrackerDataMaskAddRotationReferenceAdjusted,
        fb.TrackerDataMaskAddLinearAcceleration,
        fb.TrackerDataMaskAddTemp,
        fb.TrackerDataMaskAddTps,
        fb.TrackerDataMaskAddStayAligned,
    ):
        add(b, True)
    tracker_mask = fb.TrackerDataMaskEnd(b)

    fb.DeviceDataMaskStart(b)
    fb.DeviceDataMaskAddTrackerData(b, tracker_mask)
    fb.DeviceDataMaskAddDeviceData(b, True)
    device_mask = fb.DeviceDataMaskEnd(b)

    fb.DataFeedConfigStart(b)
    fb.DataFeedConfigAddMinimumTimeSinceLast(b, interval_ms)
    fb.DataFeedConfigAddDataMask(b, device_mask)
    config = fb.DataFeedConfigEnd(b)

    fb.StartDataFeedStartDataFeedsVector(b, 1)
    b.PrependUOffsetTRelative(config)
    feeds = b.EndVector()
    fb.StartDataFeedStart(b)
    fb.StartDataFeedAddDataFeeds(b, feeds)
    start = fb.StartDataFeedEnd(b)

    fb.DataFeedMessageHeaderStart(b)
    fb.DataFeedMessageHeaderAddMessageType(b, fb.DataFeedMessage.StartDataFeed)
    fb.DataFeedMessageHeaderAddMessage(b, start)
    return _finish_bundle(b, data_feed_hdr=fb.DataFeedMessageHeaderEnd(b))


def build_reset_request(reset_type, body_parts=None, delay=None):
    b = flatbuffers.Builder(64)
    parts_vec = None
    if body_parts:
        ids = [BODY_PART_IDS[p] for p in body_parts]
        fb.ResetRequestStartBodyPartsVector(b, len(ids))
        for i in reversed(ids):
            b.PrependUint8(i)
        parts_vec = b.EndVector()

    fb.ResetRequestStart(b)
    fb.ResetRequestAddResetType(b, getattr(fb.ResetType, reset_type))
    if parts_vec is not None:
        fb.ResetRequestAddBodyParts(b, parts_vec)
    if delay is not None:
        fb.ResetRequestAddDelay(b, delay)
    req = fb.ResetRequestEnd(b)

    fb.RpcMessageHeaderStart(b)
    fb.RpcMessageHeaderAddMessageType(b, fb.RpcMessage.ResetRequest)
    fb.RpcMessageHeaderAddMessage(b, req)
    return _finish_bundle(b, rpc_hdr=fb.RpcMessageHeaderEnd(b))



def _quat(q):
    return None if q is None else (q.W(), q.X(), q.Y(), q.Z())


def _vec(v):
    if v is None:
        return None
    out = (v.X(), v.Y(), v.Z())
    return None if any(math.isnan(c) for c in out) else out


def _parse_tracker(dev_id, hw, tr):
    info = tr.Info()
    sa = tr.StayAligned()
    tid = tr.TrackerId()
    temp = tr.Temp()
    name = b""
    if info is not None:
        name = info.CustomName() or info.DisplayName() or b""
    return TrackerSample(
        key=f"dev{dev_id}/{tid.TrackerNum() if tid else 0}",
        body_part=BODY_PART_NAMES.get(info.BodyPart(), "?") if info else "?",
        name=name.decode(errors="replace"),
        imu=IMU_NAMES.get(info.ImuType(), "?") if info else "?",
        is_imu=bool(info.IsImu()) if info else False,
        status=STATUS_NAMES.get(tr.Status(), "?"),
        quat=_quat(tr.Rotation()),
        quat_adj=_quat(tr.RotationReferenceAdjusted()),
        lin_acc=_vec(tr.LinearAcceleration()),
        temp=temp.Temp() if temp is not None else None,
        tps=tr.Tps(),
        sa_corr=sa.YawCorrectionInDeg() if sa else None,
        sa_locked=sa.Locked() if sa else None,
        sa_locked_err=sa.LockedErrorInDeg() if sa else None,
        sa_center_err=sa.CenterErrorInDeg() if sa else None,
        sa_neighbor_err=sa.NeighborErrorInDeg() if sa else None,
        battery_v=hw.BatteryVoltage() if hw else None,
        battery_pct=hw.BatteryPctEstimate() if hw else None,
        rssi=hw.Rssi() if hw else None,
    )


def parse_bundle(raw, t):
    bundle = fb.MessageBundle.GetRootAs(raw, 0)
    out = []

    for i in range(bundle.DataFeedMsgsLength()):
        hdr = bundle.DataFeedMsgs(i)
        if hdr.MessageType() != fb.DataFeedMessage.DataFeedUpdate:
            continue
        upd = fb.DataFeedUpdate()
        upd.Init(hdr.Message().Bytes, hdr.Message().Pos)
        trackers = []
        for d in range(upd.DevicesLength()):
            dev = upd.Devices(d)
            dev_id = dev.Id().Id() if dev.Id() else -1
            hw = dev.HardwareStatus()
            for j in range(dev.TrackersLength()):
                trackers.append(_parse_tracker(dev_id, hw, dev.Trackers(j)))
        out.append(FeedUpdate(t=t, trackers=trackers))

    for i in range(bundle.RpcMsgsLength()):
        hdr = bundle.RpcMsgs(i)
        if hdr.MessageType() != fb.RpcMessage.ResetResponse:
            continue
        resp = fb.ResetResponse()
        resp.Init(hdr.Message().Bytes, hdr.Message().Pos)
        out.append(ResetEvent(
            t=t,
            reset_type=RESET_TYPE_NAMES.get(resp.ResetType(), "?"),
            status=RESET_STATUS_NAMES.get(resp.Status(), "?"),
            body_parts=[BODY_PART_NAMES.get(resp.BodyParts(j), "?") for j in range(resp.BodyPartsLength())],
            progress=resp.Progress(),
            duration=resp.Duration(),
        ))

    return out



class SlimeVRClient:
    def __init__(self, url=DEFAULT_URL, interval_ms=20):
        self.url = url
        self.interval_ms = interval_ms
        self._ws = None

    async def messages(self):
        """Yields FeedUpdate / ResetEvent / ConnectionEvent forever,
        reconnecting with backoff if the server goes away or restarts."""
        backoff = 1.0
        while True:
            try:
                async with websockets.connect(self.url, max_size=None) as ws:
                    self._ws = ws
                    backoff = 1.0
                    await ws.send(build_start_data_feed(self.interval_ms))
                    yield ConnectionEvent(time.time(), True, self.url)
                    async for raw in ws:
                        if isinstance(raw, str):
                            continue
                        for msg in parse_bundle(raw, time.time()):
                            yield msg
                detail = "connection closed by the server"
            except (OSError, websockets.ConnectionClosed, websockets.InvalidHandshake) as e:
                detail = f"{type(e).__name__}: {e}"
            finally:
                self._ws = None
            yield ConnectionEvent(time.time(), False, detail)
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30.0)

    async def reset(self, reset_type="Yaw", body_parts=None, delay=None):
        """Send a ResetRequest. body_parts=None resets everything.

        Careful: SlimeVR's yaw reset aligns every tracker to the head's yaw,
        assuming a neutral pose (standing, facing forward). Lying down it
        introduces error instead of removing it.
        """
        if self._ws is None:
            raise ConnectionError("not connected to the SlimeVR server")
        await self._ws.send(build_reset_request(reset_type, body_parts, delay))

    async def reset_yaw(self, body_parts=None, delay=None):
        await self.reset("Yaw", body_parts, delay)
