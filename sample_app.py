"""
Switch Pro Controller - Interactive Terminal Sample App

Usage:
    python sample_app.py
    python sample_app.py --port COM4 --baud 115200

Press keys as mapped below. ESC to quit.
"""

import argparse
import msvcrt
import os
import sys
import time

from controller import SwitchController

# ---------------------------------------------------------------------------
# Key mappings
# ---------------------------------------------------------------------------

# Regular key char → (firmware_char, display_name)
KEY_MAP: dict[str, tuple[str, str]] = {
    "\r": ("a", "A"),           # Enter
    "\n": ("a", "A"),
    "\x08": ("b", "B"),         # Backspace
    "\t": ("x", "X"),           # Tab
    "\\": ("y", "Y"),           # backslash
    "[":  ("l", "L"),
    "]":  ("r", "R"),
    ",":  ("z", "ZL"),
    ".":  ("c", "ZR"),
    "=":  ("+", "Plus"),
    "-":  ("-", "Minus"),
}

# Second byte after 0xe0 / 0x00 prefix → (firmware_char, display_name)
SPECIAL_MAP: dict[str, tuple[str, str]] = {
    "H": ("w", "D-pad Up"),     # Arrow Up
    "P": ("s", "D-pad Down"),   # Arrow Down
    "K": ("q", "D-pad Left"),   # Arrow Left
    "M": ("e", "D-pad Right"),  # Arrow Right
    "G": ("h", "HOME"),         # Home key
    "S": ("p", "Capture"),      # Delete key
}

HELP_TEXT = """\
┌─────────────────────────────────────────────────┐
│  Nintendo Switch Pro Controller - Terminal App  │
├───────────────┬─────────────────────────────────┤
│  Arrow keys   │  D-pad                          │
│  Enter        │  A                              │
│  Backspace    │  B                              │
│  Tab          │  X                              │
│  \\            │  Y                              │
│  [ / ]        │  L / R                          │
│  , / .        │  ZL / ZR                        │
│  = / -        │  + / −                          │
│  Home         │  HOME                           │
│  Delete       │  Capture                        │
├───────────────┴─────────────────────────────────┤
│  H            │  Show this help                 │
│  ESC / Ctrl-C │  Quit                           │
└─────────────────────────────────────────────────┘
"""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def clear_status():
    """Overwrite the status line."""
    sys.stdout.write("\r" + " " * 60 + "\r")
    sys.stdout.flush()


def show_status(msg: str):
    sys.stdout.write(f"\r> {msg:<56}")
    sys.stdout.flush()


def read_key() -> str | None:
    """
    Read one logical keypress from the Windows console.
    Returns the firmware character to send, or None for ignored keys.
    Raises SystemExit on ESC / Ctrl-C.
    """
    ch = msvcrt.getwch()

    # ESC or Ctrl-C → quit
    if ch in ("\x1b", "\x03"):
        raise SystemExit(0)

    # Show help
    if ch in ("H", "h"):
        print()
        print(HELP_TEXT)
        show_status("Waiting for input...")
        return None

    # Special keys (arrows, Home, Del, …)
    if ch in ("\xe0", "\x00"):
        ch2 = msvcrt.getwch()
        entry = SPECIAL_MAP.get(ch2)
        if entry:
            return entry  # (firmware_char, display_name)
        return None

    entry = KEY_MAP.get(ch)
    if entry:
        return entry  # (firmware_char, display_name)

    # Printable fallback – show the unmapped key as a hint
    if ch.isprintable():
        show_status(f"Unmapped key: {ch!r}  (press H for help)")
    return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Interactive Switch controller via UART")
    parser.add_argument("--port",  default="COM4",   help="Serial port (default: COM4)")
    parser.add_argument("--baud",  default=115200, type=int, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    print(HELP_TEXT)

    try:
        con = SwitchController(args.port, args.baud)
    except Exception as e:
        print(f"[ERROR] Could not open {args.port}: {e}")
        sys.exit(1)

    print(f"Connected to {args.port} at {args.baud} baud.\n")
    show_status("Waiting for input...")

    try:
        with con:
            while True:
                result = read_key()
                if result is None:
                    continue
                firmware_char, display_name = result
                con.press(firmware_char)
                show_status(f"Sent: {display_name}")

    except SystemExit:
        clear_status()
        print("\nBye.")
    except KeyboardInterrupt:
        clear_status()
        print("\nBye.")


if __name__ == "__main__":
    main()
