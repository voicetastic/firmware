# PlatformIO pre-build hook (OPT-IN): cross-compile voicetastic-esp32-bridge
# from a sibling voicetastic-core checkout and link its static archive + C
# header into the firmware. Only referenced by the `t-deck-tft-vtcore` env, so
# the default builds (and CI) never invoke cargo.
#
# UNVERIFIED ON HARDWARE - slice 1 toolchain proof. Validate with:
#   espup install                       # one-time Xtensa Rust toolchain
#   rustup target add ... (via espup)
#   pio run -e t-deck-tft-vtcore        # core sibling at ../voicetastic-core
#
# The open question this first run answers (see the bridge crate README): does
# the `xtensa-esp32s3-espidf` (std) archive link cleanly into the Arduino-ESP32
# build, or does esp-idf-sys's ownership of ESP-IDF force a `no_std` core subset
# on `xtensa-esp32s3-none-elf`.

import os
import subprocess

Import("env")  # noqa: F821  (injected by PlatformIO/SCons)

TARGET = "xtensa-esp32s3-espidf"
CRATE = "voicetastic-esp32-bridge"

project_dir = env["PROJECT_DIR"]  # noqa: F821
# Mirror the device-ui sibling convention: core lives next to the firmware
# checkout. Override with VT_CORE_DIR.
core_dir = os.environ.get(
    "VT_CORE_DIR",
    os.path.normpath(os.path.join(project_dir, "..", "voicetastic-core")),
)
if not os.path.isdir(core_dir):
    raise SystemExit(
        f"[vt-core] voicetastic-core not found at {core_dir}; "
        "set VT_CORE_DIR or place it as a sibling of the firmware checkout."
    )

print(f"[vt-core] building {CRATE} ({TARGET}) from {core_dir}")
subprocess.check_call(
    ["cargo", "build", "--release", "-p", CRATE, "--target", TARGET],
    cwd=core_dir,
)

lib_dir = os.path.join(core_dir, "target", TARGET, "release")
lib_path = os.path.join(lib_dir, "libvoicetastic_esp32_bridge.a")
include_dir = os.path.join(core_dir, "crates", CRATE, "include")
if not os.path.isfile(lib_path):
    raise SystemExit(f"[vt-core] expected archive missing: {lib_path}")

env.Append(CPPPATH=[include_dir])               # noqa: F821  vt_core header
env.Append(LIBPATH=[lib_dir])                   # noqa: F821
env.Append(LIBS=["voicetastic_esp32_bridge"])   # noqa: F821  links the .a
print(f"[vt-core] linked {lib_path}")
