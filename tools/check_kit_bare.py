"""Minimal-repro ladder, level 0: does Kit itself come up at all, with none of OmniGibson's own
extension set (physics, robotics, etc.) loaded?

    python tools/check_kit_bare.py

Boots `isaacsim.SimulationApp` directly, with no `experience=` kit-file override -- so it uses
whichever default Isaac Sim application the pip install ships, not OmniGibson's much larger
`omnigibson_4_5_0.kit` (which pulls in physics, robot motion, replicator, etc. on top). If this
still hits the same quiet-exit or crash, the failure is in Kit's own core (extension registration,
rendering) and has nothing to do with anything OmniGibson adds. If this succeeds cleanly, the next
rung (`check_og_minimal.py`) narrows it down further.

Companion to `docs/widowxai_bringup_status.md`'s "2026-09-30 environment incident" -- see the
minimal-repro-ladder plan recorded there before adding further rungs.
"""
import os
import sys
import time

sys.path.insert(0, ".")


def main():
    # simulator.py sets this before importing isaacsim so the EULA prompt (an interactive
    # input() call in omni/kit_app.py) doesn't block a headless run -- needed here too since
    # this script imports isaacsim directly rather than through OmniGibson's own bring-up.
    os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

    print("[check_kit_bare] importing isaacsim...", flush=True)
    from isaacsim import SimulationApp

    t0 = time.time()
    print("[check_kit_bare] constructing SimulationApp(headless=True)...", flush=True)
    app = SimulationApp({"headless": True})
    print(f"[check_kit_bare] SimulationApp constructed OK after {time.time() - t0:.1f}s", flush=True)

    # Step a handful of frames so we exercise the render loop briefly, the same way a real
    # og.Environment() would immediately after startup -- not just construct-and-close.
    for i in range(30):
        app.update()
    print(f"[check_kit_bare] 30 app.update() calls done after {time.time() - t0:.1f}s total", flush=True)

    app.close()
    print("[check_kit_bare] app.close() returned -- SUCCESS", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
