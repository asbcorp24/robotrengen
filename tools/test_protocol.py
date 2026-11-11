"""Build/run the actual C++ wire implementation on the host, no ESP32 needed."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--payload", action="store_true",
                        help="Also test JSON/CBOR conversion after building Blue Pill")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    if not compiler:
        raise SystemExit("Install g++/clang++ or set CXX to the compiler executable.")
    protocol = root / "lib/BridgeProtocol/src"
    flags = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", "-O2",
             "-I", str(protocol)]
    tests = [[str(protocol / "BridgeProtocol.cpp"), str(root / "test/host/protocol_test.cpp")]]
    if args.payload:
        arduino_json = root / ".pio/libdeps/bluepill_f103c8/ArduinoJson/src"
        if not (arduino_json / "ArduinoJson.h").is_file():
            raise SystemExit("Run pio run -e bluepill_f103c8 to install ArduinoJson.")
        payload = root / "lib/BridgePayload/src"
        tests.append(["-I", str(payload), "-I", str(arduino_json),
                      str(payload / "BridgePayload.cpp"), str(root / "test/host/payload_test.cpp")])
    with tempfile.TemporaryDirectory(prefix="crawler-protocol-") as directory:
        binary = Path(directory) / ("protocol_test.exe" if os.name == "nt" else "protocol_test")
        for sources in tests:
            subprocess.run([*flags, *sources, "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
    subprocess.run([sys.executable, str(root / "test/host/test_simulator.py")], check=True)


if __name__ == "__main__":
    main()
