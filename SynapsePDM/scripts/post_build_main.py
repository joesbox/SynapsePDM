import hashlib
from pathlib import Path
import configparser
import subprocess
import sys

# --- Get directories from command line ---
build_dir = Path(sys.argv[1])
src_dir = Path(sys.argv[2])

# --- Get FW version from globals.h if available ---
version = None
globals_h = src_dir / "src/globals.h"

if globals_h.exists():
    with open(globals_h, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith('#define FW_VER'):
                version = line.split('"')[1]  # extract string between quotes
                break

# --- Fallback: platformio.ini or git describe ---
if not version:
    ini_path = src_dir / "platformio.ini"
    config = configparser.ConfigParser()
    config.read(ini_path)

    if "env:myboard" in config and "build_flags" in config["env:myboard"]:
        flags = config["env:myboard"]["build_flags"]
        for flag in flags.split():
            if flag.startswith("-DVERSION="):
                version = flag.split("=")[1].strip('"')
                break

if not version:
    try:
        version = subprocess.check_output(
            ["git", "describe", "--tags"], cwd=src_dir
        ).decode().strip()
    except Exception:
        version = "v0.0.0"

print(f"[POST-BUILD] Version: {version}")

# --- Input firmware ---
firmware_bin = build_dir / "firmware.bin"
if not firmware_bin.exists():
    print(f"[POST-BUILD] Firmware not found: {firmware_bin}")
    exit(0)

# --- Output filenames with version ---
bin_out = build_dir / f"SynapsePDM-{version}.bin"
sha_out = build_dir / f"SynapsePDM-{version}.sha256"
sig_out = build_dir / f"SynapsePDM-{version}.sig"

# Copy / rename original firmware
firmware_bin.replace(bin_out)
print(f"[POST-BUILD] Firmware -> {bin_out}")

# --- Generate SHA256 & signature only if key exists ---
private_key_path = src_dir / "keys/private.key"
if private_key_path.exists():
    try:
        from nacl.signing import SigningKey

        with open(private_key_path, "rb") as f:
            sk_data = f.read()
        signing_key = SigningKey(sk_data)

        with open(bin_out, "rb") as f:
            data = f.read()

        sha256_hash = hashlib.sha256(data).hexdigest()
        with open(sha_out, "w") as f:
            f.write(sha256_hash)
        print(f"[POST-BUILD] SHA256 -> {sha_out}")

        # Signature
        sig = signing_key.sign(hashlib.sha256(data).digest()).signature
        with open(sig_out, "wb") as f:
            f.write(sig)
        print(f"[POST-BUILD] Firmware signed -> {sig_out}")

    except Exception as e:
        print(f"[POST-BUILD] Signing failed: {e}")
else:
    print("[POST-BUILD] No private key found, skipping SHA256 and signing.")