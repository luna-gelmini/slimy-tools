"""Send a notification from the Pi, without needing anything open on the PC.

Backends, picked with --notify:
    log       only stdout (journalctl when run as a service)
    ntfy      POST to https://ntfy.sh/<topic>; subscribe in the app or browser
    discord   POST to a Discord webhook URL

Both ntfy and Discord are outbound HTTPS, so no ports have to be opened and
messages still arrive when the PC is off.
"""

import json
import time
import urllib.error
import urllib.request

TIMEOUT = 15


def log(msg):
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {msg}", flush=True)


def _post(url, data, headers):
    req = urllib.request.Request(url, data=data, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
            return 200 <= r.status < 300
    except (urllib.error.URLError, OSError) as e:
        log(f"notification failed ({url.split('/')[2]}): {e}")
        return False


def send(title, message, backend="log", ntfy_topic=None, discord_webhook=None, priority="default"):
    log(f"[{title}] {message}")
    ok = True
    if backend in ("ntfy", "both") and ntfy_topic:
        ok &= _post(
            f"https://ntfy.sh/{ntfy_topic}",
            message.encode("utf-8"),
            {"Title": title, "Priority": priority, "Tags": "satellite"},
        )
    if backend in ("discord", "both") and discord_webhook:
        ok &= _post(
            discord_webhook,
            json.dumps({"content": f"**{title}**\n{message}"}).encode("utf-8"),
            {"Content-Type": "application/json"},
        )
    return ok


def add_arguments(ap):
    ap.add_argument("--notify", choices=["log", "ntfy", "discord", "both"], default="log")
    ap.add_argument("--ntfy-topic", help="ntfy.sh topic name, e.g. luna-slimevr-7f3a")
    ap.add_argument("--discord-webhook", help="Discord webhook URL")


def sender(args):
    def send_it(title, message, priority="default"):
        return send(title, message, args.notify, args.ntfy_topic, args.discord_webhook, priority)

    return send_it
