# Third-party code

- `firmware/*.patch` apply on top of [SlimeVR-Tracker-ESP](https://github.com/SlimeVR/SlimeVR-Tracker-ESP)
  v0.7.3 (MIT). The patches are diffs, not a copy of that tree; build them
  against a checkout of the upstream tag.
- `solarxr_fb.py` is generated from the [SolarXR-Protocol](https://github.com/SlimeVR/SolarXR-Protocol)
  schema (MIT / Apache-2.0) with `flatc --python --gen-all --gen-onefile`.
