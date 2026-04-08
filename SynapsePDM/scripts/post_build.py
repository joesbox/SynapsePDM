# scripts/post_build.py
from SCons.Script import DefaultEnvironment
import subprocess
import sys
from pathlib import Path

env = DefaultEnvironment()

def post_build_action(source, target, env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    build_dir = Path(env.subst("$BUILD_DIR"))
    firmware_bin = build_dir / "firmware.bin"

    # Pass directories as arguments to main script
    main_script = project_dir / "scripts/post_build_main.py"
    subprocess.run([sys.executable, str(main_script),
                    str(build_dir), str(project_dir)], check=True)

# Hook it to firmware.bin
firmware_bin = Path(env.subst("$BUILD_DIR")) / "firmware.bin"
env.AddPostAction(str(firmware_bin), post_build_action)