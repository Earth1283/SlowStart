import subprocess
import sys
import time
from pathlib import Path
import os

from rich import print
from rich.console import Console
from rich.theme import Theme
from rich.panel import Panel

main_theme = Theme({
    "warning_explicit": "#FF0000 on #FFFF00",
    "success": "green bold",
    "warning": "yellow",
    "error": "bold #FF0000"
})

console = Console(theme=main_theme)

def find_adb(search_dir: Path | None = None) -> bool | Path:
    targets = {"adb.exe", "adb"} if sys.platform == "win32" else {"adb"}
    local_dir = search_dir or Path.cwd()

    for root, _, files in os.walk(local_dir):
        for file in files:
            is_match = (
                file.lower() in targets
                if sys.platform == "win32"
                else file in targets
            )

            if is_match:
                full_path = Path(root) / file
                if os.access(full_path, os.X_OK) or sys.platform == "win32":
                    return full_path

    return False

def adb(adb_path: Path, *args: str) -> str:
    result = subprocess.run([str(adb_path), *args], capture_output=True, text=True)
    output = result.stdout.strip()
    if output:
        return output
    return result.stderr.strip() or str(result.returncode)

def parse_devices(output:str) -> list:
    devices = []
    for line in output.splitlines():
        parts = line.strip().split()
        if len(parts) == 2:
            devices.append({"device_id": parts[0], "status": parts[1]})
    return devices

def main():
    console.clear()
    console.print("I see that we're rooting your Control Hub today.")
    console.print("[#FF0000 on #FFFF00]To be clear, the author, me, is NOT LIABLE for any damages done to your hardware[/]")
    user = console.input("Should we begin? Answer with yes/no :thinking_face:\n> ").strip().lower()
    if user not in ("yes", "y"):
        console.print("Alright, goodbye :)")
        sys.exit(0)
    should_specify_path = False
    adb_path = None
    with console.status("[blue]Looking for your ADB binary in the local directory...", spinner="aesthetic"):
        found = find_adb()
        if isinstance(found, Path):
            console.print("[#90ee90]ADB binary found!")
            adb_path = found
            should_specify_path = False
        else:
            console.print("[#FF0000]Error: ADB binary not found.")
            should_specify_path = True
    if should_specify_path:
        while True:
            raw = console.input("Please enter the [bold]absolute path[/bold] to your ADB binary (leave blank if you don't have one): \n> ").strip()
            if not raw:
                console.print(
                    "[error]No ADB binary available. Download platform-tools from "
                    "[link]https://developer.android.com/tools/releases/platform-tools[/link] and try again."
                )
                sys.exit(1)
            candidate = Path(raw)
            if not candidate.exists():
                console.print(f"[error]Error: '{candidate}' doesn't exist. Try again.")
                continue

            targets = {"adb.exe", "adb"} if sys.platform == "win32" else {"adb"}
            is_adb_binary = (
                candidate.is_file()
                and candidate.name in targets
                and (os.access(candidate, os.X_OK) or sys.platform == "win32")
            )

            if is_adb_binary:
                adb_path = candidate
                break
            elif candidate.is_dir():
                found_in_dir = find_adb(candidate)
                if isinstance(found_in_dir, Path):
                    console.print(f"[#90ee90]ADB binary found in '{candidate}'!")
                    adb_path = found_in_dir
                    break
                console.print(f"[error]No ADB binary found in '{candidate}'. Try again.")
            else:
                console.print(f"[error]Error: '{candidate}' isn't an ADB binary or a folder containing one. Try again.")

    assert adb_path is not None

    console.print(f"Using ADB binary at [bold]{adb_path}[/bold]")

    console.print(
        Panel(
            "[warning_explicit]This is your LAST WARNING.[/]\n"
            "We are about to run code that may affect your Control Hub, aka your Robot.\n"
            "This is the last chance to abort before anything happens.",
            title="[bold red]LAST WARNING[/bold red]",
            border_style="red",
        )
    )
    confirm = console.input("Type [bold]yes[/bold] to proceed, anything else to abort: \n> ").strip().lower()
    if confirm not in ("yes", "y"):
        console.print("Aborted. Goodbye :)")
        sys.exit(0)
    
    # Actual business logic
    # First, check active devices
    while True:
        with console.status("[blue]Checking for connected devices...", spinner="aesthetic"):
            device_list = parse_devices(adb(adb_path, "devices"))
        statuses = {d["status"] for d in device_list}
        if "device" in statuses:
            console.print("[#90ee90]Device found.")
            break
        if "unauthorized" in statuses:
            console.print(
                "[warning]Device detected but unauthorized. Check your Control Hub's screen "
                "and accept the 'Allow USB debugging' prompt.[/]"
            )
        if "offline" in statuses:
            adb(adb_path, "disconnect")
        console.print("[#87ceeb]Please connect your Control Hub via USB/Type C, and press enter to try again")
        console.input("Press enter when done... ")

    # Then, enable port 5555 for ADB
    with console.status("[blue]Enabling TCP/IP mode on port 5555...", spinner="aesthetic"):
        tcpip_result = str(adb(adb_path, "tcpip", "5555"))
    if "restarting in tcp" not in tcpip_result.lower() and "0" not in tcpip_result.lower():
        console.print(
            Panel(
                f"[error]Failed to enable TCP/IP mode on port 5555.[/]\nADB said: {tcpip_result}",
                title="[bold red]Error[/bold red]",
                border_style="red",
            )
        )
        sys.exit(1)

    console.print("[#87ceeb]Opened port 5555 for ADB")

    # Attempt adb root
    # First call usually just reports "restarting adbd as root" - the daemon
    # needs a sec to breathe
    rooted = False
    with console.status("[blue]Attempting to root Control Hub...", spinner="aesthetic"):
        for _ in range(10):
            if adb(adb_path, "root") == "adbd is already running as root":
                rooted = True
                break
            time.sleep(1)
    if not rooted:
        console.print(
            Panel(
                "[error]Failed to restart adbd as root.[/]\n"
                "Your Control Hub firmware may not support root, or a driver/connection issue occurred.",
                title="[bold red]Error[/bold red]",
                border_style="red",
            )
        )
        sys.exit(1)

    console.print("[#90EE90]Rooted the Control Hub, re-mounting system paths...")
    with console.status("[blue]Re-mounting system paths...", spinner="aesthetic"):
        remount_result = adb(adb_path, "remount")
    if remount_result == "remount succeeded":
        console.print("[#90EE90]Successfully re-mounted system paths")
    else:
        console.print(
            Panel(
                "[error]Failed to re-mount system paths as read-write.[/]\n"
                "We may not actually have root access.",
                title="[bold red]Error[/bold red]",
                border_style="red",
            )
        )
        sys.exit(1)

    console.print("[#87ceeb]Checking one last time if we really got root...")
    with console.status("[blue]Verifying root...", spinner="aesthetic"):
        whoami_result = adb(adb_path, "shell", "whoami")
        whoami_result = whoami_result.lower().strip()
    if whoami_result == "root":
        console.print("[#90ee90]We have verified that we are root.")
    else:
        console.print(
            Panel(
                "[warning]'whoami' did not return 'root'.[/]\n"
                "Root access may not be fully active. Proceed with caution.",
                title="[bold yellow]Warning[/bold yellow]",
                border_style="yellow",
            )
        )

    # Final step: let the user pick what to do now that we're rooted
    console.print(
        Panel(
            "1. Enter interactive ADB shell\n"
            "2. Persist ADB-over-TCP across reboots\n"
            "3. Exit",
            title="What next?",
        )
    )
    choice = console.input("Choose an option (1/2/3): \n> ").strip()
    if choice == "1":
        console.print("[#87ceeb]Entering interactive shell. Type 'exit' to quit.")
        while True:
            shell_cmd = console.input("root@yourFtcBot\n❯ ").strip()
            if not shell_cmd:
                continue
            if shell_cmd.lower() in ("exit", "quit"):
                break
            print(adb(adb_path, "shell", shell_cmd))
    elif choice == "2":
        with console.status("[blue]Persisting ADB-over-TCP...", spinner="aesthetic"):
            adb(adb_path, "shell", "echo 'persist.adb.tcp.port=5555' >> /vendor/build.prop")
            adb(adb_path, "shell \"setprop persist.adb.tcp.port 5555\"")
            adb(adb_path, "shell sync")
        console.print("[#90ee90]ADB-over-TCP will now persist across reboots.")
    else:
        console.print("Goodbye :)")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n[warning]Interrupted. Goodbye :)[/]")
        sys.exit(1)
