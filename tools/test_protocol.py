"""Build/run the actual C++ wire implementation on the host, no ESP32 needed."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    if not compiler:
        raise SystemExit("Install g++/clang++ or set CXX to the compiler executable.")
    with tempfile.TemporaryDirectory(prefix="crawler-protocol-") as directory:
        binary = Path(directory) / ("protocol_test.exe" if os.name == "nt" else "protocol_test")
        subprocess.run([
            compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", "-O2",
            "-I", str(root / "lib/BridgeProtocol/src"),
            str(root / "lib/BridgeProtocol/src/BridgeProtocol.cpp"),
            str(root / "test/host/protocol_test.cpp"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
    subprocess.run([sys.executable, str(root / "test/host/test_simulator.py")], check=True)


if __name__ == "__main__":
    main()
