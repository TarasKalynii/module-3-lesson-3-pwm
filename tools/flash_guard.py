"""Build, flash and optionally monitor this project's ESP32-S3 over native USB."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build"
BOARD_SERIAL = "E8:3D:C1:F2:9D:A4"
USB_ID = (0x303A, 0x1001)


def board_port(ports):
    matches = [p for p in ports if (p.vid, p.pid) == USB_ID
               and (p.serial_number or "").upper() == BOARD_SERIAL]
    if len(matches) != 1:
        raise RuntimeError("Не знайдено саме цю ESP32-S3. Перевір USB-підключення плати.")
    return matches[0].device.replace("/dev/tty.", "/dev/cu.")


def port_holders(port):
    paths = [port, port.replace("/dev/cu.", "/dev/tty.")]
    result = subprocess.run(["/usr/sbin/lsof", "-t", *paths],
                            capture_output=True, text=True)
    if result.returncode not in (0, 1):
        raise RuntimeError("Не вдалося перевірити власника USB-порту: " + result.stderr)
    return {int(p) for p in result.stdout.split()}


def prepare():
    ports = list(list_ports.comports())
    port = board_port(ports)
    holders = port_holders(port)
    rows = subprocess.check_output(["/bin/ps", "-axo", "pid=,command="], text=True)
    only_one_esp = sum((p.vid, p.pid) == USB_ID for p in ports) == 1
    for row in rows.splitlines():
        fields = row.strip().split(None, 1)
        if len(fields) != 2:
            continue
        pid_text, command = fields
        pid = int(pid_text)
        monitor = (pid in holders and str(ROOT) in command
                   and ("esp_idf_monitor" in command or "idf_monitor.py" in command))
        openocd = (only_one_esp and "/openocd " in command
                   and "board/esp32s3-builtin.cfg" in command)
        if monitor or openocd:
            print("Закриваю монітор плати." if monitor else "Зупиняю OpenOCD цієї плати.", flush=True)
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + 3
    while port_holders(port):
        if time.monotonic() >= deadline:
            raise RuntimeError("Порт зайнятий іншою програмою. Її не закрито автоматично. "
                               f"Власники порту: {sorted(port_holders(port))}")
        time.sleep(0.1)
    settings_path = ROOT / ".vscode/settings.json"
    settings = json.loads(settings_path.read_text())
    updates = {"idf.port": port, "idf.monitorPort": port, "idf.flashType": "UART"}
    if any(settings.get(key) != value for key, value in updates.items()):
        settings.update(updates)
        settings_path.write_text(json.dumps(settings, indent=2, ensure_ascii=False) + "\n")
    print(f"ESP32-S3 {BOARD_SERIAL}: {port}", flush=True)
    return port


def run(command, *, cwd=ROOT, log=None):
    output = []
    with subprocess.Popen(command, cwd=cwd, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, errors="replace") as process:
        try:
            for line in process.stdout:
                print(line, end="", flush=True)
                if log:
                    log.write(line)
                    log.flush()
                output.append(line)
            return process.wait(), "".join(output)
        except BaseException:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            raise


def monitor(idf, port):
    # IDF monitor needs the terminal itself for Ctrl+] and Ctrl+T. Do not pipe it
    # through run(), and do not reset back into the USB download boot mode.
    return subprocess.call([*idf, "-p", port, "monitor", "--no-reset"], cwd=ROOT)


def wait_for_board(timeout=10):
    # A watchdog reset can briefly disconnect/re-enumerate the native USB port.
    deadline = time.monotonic() + timeout
    while True:
        try:
            return board_port(list(list_ports.comports()))
        except RuntimeError:
            if time.monotonic() >= deadline:
                raise RuntimeError("Після прошивання USB-порт плати не з'явився.")
            time.sleep(0.2)


def flash_command(port, manifest):
    if manifest["extra_esptool_args"]["chip"] != "esp32s3":
        raise RuntimeError("Збірка призначена для іншого чипа; прошивання скасовано.")
    command = [sys.executable, "-m", "esptool", "--chip", "esp32s3",
               "--port", port, "--baud", "115200", "--before", "usb-reset",
               "--after", "no-reset-stub", "--connect-attempts", "3", "write-flash",
               *manifest["write_flash_args"], "--skip-flashed"]
    for address, filename in manifest["flash_files"].items():
        image = (BUILD / filename).resolve()
        if not image.is_relative_to(BUILD.resolve()) or not image.is_file():
            raise RuntimeError(f"Не знайдено файл прошивки у build: {filename}")
        command.extend([address, str(image)])
    return command


def start_application_command(port):
    # ESP32-S3 RTC_CNTL_OPTION1_REG bit 0 forces USB download mode. Clear only
    # that volatile bit before watchdog reset, as esptool's hard_reset() does.
    # This does not alter flash or eFuses. Avoid toggling USB RTS/DTR here.
    return [sys.executable, "-m", "esptool", "--chip", "esp32s3",
            "--port", port, "--baud", "115200", "--before", "no-reset",
            "--after", "watchdog-reset", "--connect-attempts", "3",
            "write-mem", "0x6000812c", "0", "0x1"]


def main(action):
    BUILD.mkdir(exist_ok=True)
    # Held across build + flash; the OS releases the lock even if the task is stopped.
    with (BUILD / ".flash.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Уже виконується збірка або прошивання. Дочекайся завершення.")
        if action == "prepare":
            prepare()
            return 0
        idf = [sys.executable, str(Path(os.environ["IDF_PATH"]) / "tools/idf.py")]
        if action == "monitor":
            port = prepare()
            # Monitoring has its own task and must not hold the build/flash lock.
            fcntl.flock(lock, fcntl.LOCK_UN)
            return monitor(idf, port)
        if action in ("flash", "flash-monitor"):
            prepare()
        log_dir = BUILD / "flash-logs"
        log_dir.mkdir(exist_ok=True)
        log_path = log_dir / (time.strftime("%Y%m%d-%H%M%S") + ".log")
        with log_path.open("w") as log:
            print(f"Журнал: {log_path}", flush=True)
            result, _ = run([*idf, "build"], log=log)
            if result or action == "build":
                return result
            manifest = json.loads((BUILD / "flasher_args.json").read_text())
            for attempt in range(2):
                port = prepare()
                result, output = run(flash_command(port, manifest), cwd=BUILD, log=log)
                if result == 0:
                    result, _ = run(start_application_command(port), log=log)
                    if result:
                        print("Прошивку записано, але автоматичний запуск не вдався. Дивись журнал.", flush=True)
                        return result
                    if action == "flash-monitor":
                        port = wait_for_board()
                        print("Прошивання успішне. Відкриваю логи SAFE / HEARTBEAT. Вихід: Ctrl+].", flush=True)
                        fcntl.flock(lock, fcntl.LOCK_UN)
                        return monitor(idf, port)
                    print("Прошивання успішне. Програму запущено, монітор не відкрито.", flush=True)
                    return 0
                transient = any(message in output.lower() for message in
                                ("no serial data received", "could not open", "serial data stream stopped"))
                if not transient or attempt == 1:
                    break
                print("Повторюю USB-з'єднання один раз.", flush=True)
                time.sleep(1)
            print("Плата не прийняла прошивку. Якщо немає відповіді: вимкни живлення двигуна, "
                  "затисни BOOT, натисни RESET/EN, відпусти BOOT і повтори дію.", flush=True)
            return result


def interrupt(*_):
    raise KeyboardInterrupt


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["prepare", "build", "flash", "flash-monitor", "monitor"], default="flash-monitor", nargs="?")
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupt)
    try:
        sys.exit(main(args.action))
    except KeyboardInterrupt:
        print("\nДію зупинено; USB-порт звільнено.", flush=True)
        sys.exit(130)
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f"Помилка: {error}", file=sys.stderr)
        sys.exit(1)
