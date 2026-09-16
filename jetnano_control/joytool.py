"""Show what pygame sees: every joystick with its live axes, buttons and hats.

Run it with ``jetnano-joy`` on the operator machine to find the axis and button
numbers for the config file. It works over SSH; no display is needed.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List, Optional


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="jetnano-joy", description="Print live joystick axes and buttons.")
    parser.add_argument("--interval", type=float, default=0.2, help="seconds between updates")
    args = parser.parse_args(argv)
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame
    pygame.init()
    pygame.joystick.init()
    try:
        while True:
            pygame.event.pump()
            count = pygame.joystick.get_count()
            lines = [f"joysticks: {count}   (Ctrl-C to quit)"]
            for index in range(count):
                joy = pygame.joystick.Joystick(index)
                joy.init()
                axes = " ".join(f"{i}:{joy.get_axis(i):+.2f}" for i in range(joy.get_numaxes()))
                buttons = "".join("#" if joy.get_button(i) else "." for i in range(joy.get_numbuttons()))
                hats = " ".join(str(joy.get_hat(i)) for i in range(joy.get_numhats()))
                lines.append(f"[{index}] {joy.get_name()}")
                lines.append(f"    axes    {axes}")
                lines.append(f"    buttons {buttons}  (index 0 is leftmost)")
                if hats:
                    lines.append(f"    hats    {hats}")
            sys.stdout.write("\033[2J\033[H" + "\n".join(lines) + "\n")
            sys.stdout.flush()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
