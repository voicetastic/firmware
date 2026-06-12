# PlatformIO pre-build hook (OPT-IN): cross-compile voicetastic-esp32-bridge
# (no_std, built from voicetastic-proto) for the device and link its static
# archive + C header into the firmware. Only referenced by the
# `t-deck-tft-vtcore` env, so default builds / CI never run cargo.
#
# Deterministic: the bridge is built from voicetastic-core pinned at
# VT_CORE_REV - NOT "whatever branch a sibling checkout happens to be on". For
# local iteration, set VT_CORE_DIR to a working checkout; the script warns if
# its HEAD differs from the pin.
#
# Requires the espup Xtensa Rust toolchain (`cargo +esp`, ships rust-src for
# -Zbuild-std). The firmware provides `memalign`/`free` (newlib) that the
# bridge's global allocator binds to.

import os
import subprocess

Import("env")  # noqa: F821  (injected by PlatformIO/SCons)

TARGET = "xtensa-esp32s3-none-elf"
CRATE = "voicetastic-esp32-bridge"
CORE_REPO = "https://github.com/voicetastic/voicetastic-core.git"
# Pinned rev: no_std bridge over voicetastic-proto (branch feat/voicetastic-proto).
# Repoint to a `main` tag/rev once the proto + bridge PRs land.
VT_CORE_REV = "9051085fbbeedb7bfc159acf63ed1cfde8281462"

project_dir = env["PROJECT_DIR"]  # noqa: F821
core_dir = os.environ.get("VT_CORE_DIR")
if core_dir:
    head = subprocess.check_output(["git", "-C", core_dir, "rev-parse", "HEAD"]).decode().strip()
    if head != VT_CORE_REV:
        print(f"::warning::[vt-core] VT_CORE_DIR HEAD {head[:10]} != pinned {VT_CORE_REV[:10]}")
else:
    core_dir = os.path.join(project_dir, ".pio", "voicetastic-core")
    if not os.path.isdir(os.path.join(core_dir, ".git")):
        subprocess.check_call(["git", "clone", CORE_REPO, core_dir])
    subprocess.check_call(["git", "-C", core_dir, "fetch", "--quiet", "origin", VT_CORE_REV])
    subprocess.check_call(["git", "-C", core_dir, "checkout", "--quiet", VT_CORE_REV])

print(f"[vt-core] building {CRATE} ({TARGET}) @ {VT_CORE_REV[:10]} from {core_dir}")
subprocess.check_call(
    [
        "cargo", "+esp", "build", "--release",
        "-Zbuild-std=core,alloc",
        "-p", CRATE,
        "--target", TARGET,
    ],
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
