"""
Run this ONCE, while your laptop is still on your normal Wi-Fi/internet —
i.e. BEFORE you connect to the Pluto drone's camera Wi-Fi network.

It just triggers Ultralytics to download and cache the small ReID/
classification weights that BoT-SORT's ReID mode (with_reid: true in
botsort_reid.yaml) needs. Once this succeeds, the weights file will sit
in this same folder (or Ultralytics' local cache), so the main detection
script can load it from disk with zero network access — which matters
because once you're joined to the Pluto's Wi-Fi, there IS no internet.

Usage:
    python setup_reid_model.py

You only need to run this again if you delete the cached weights file,
switch to a different machine, or Ultralytics changes the default model.
"""

from ultralytics import YOLO

print("Downloading/caching ReID model weights (needs internet)...")
YOLO("yolo11n-cls.pt")
print("Done. This file is now cached locally.")
print("You can now connect to the Pluto's Wi-Fi and run main.py normally —")
print("no internet will be needed at that point.")
