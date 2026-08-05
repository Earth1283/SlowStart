from __future__ import annotations
import os
import shlex
import subprocess
import sys
from pathlib import Path
import argparse

from rich.console import Console
from rich.table import Table

console = Console()

CONTROL_HUB_IP = "192.168.43.1"
CONTROL_HUB_PORT = "5555"


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


def find_gradlew(search_dir: Path | None = None) -> Path | None:
    target = "gradlew.bat" if sys.platform == "win32" else "gradlew"
    local_dir = search_dir or Path.cwd()

    for root, _, files in os.walk(local_dir):
        if target in files:
            full_path = Path(root) / target
            if sys.platform == "win32" or os.access(full_path, os.X_OK):
                return full_path

    return None


def adb(adb_path: Path, *args: str) -> str:
    result = subprocess.run([str(adb_path), *args], capture_output=True, text=True)
    output = result.stdout.strip()
    if output:
        return output
    return result.stderr.strip() or str(result.returncode)


def run_live(cmd: list[str], cwd: Path | None = None) -> int:
    result = subprocess.run(cmd, cwd=cwd)
    return result.returncode


def parse_devices(output: str) -> list:
    devices = []
    for line in output.splitlines():
        parts = line.strip().split()
        if len(parts) == 2:
            devices.append({"device_id": parts[0], "status": parts[1]})
    return devices


def step(msg: str) -> None:
    console.print(f"[dim]→[/] {msg}")


def require_adb() -> Path:
    step("searching for adb executable in local directory...")
    adb_path = find_adb()
    if not adb_path:
        console.print("[bold red]✗ Error:[/] no adb executable found in local directory.")
        sys.exit(1)
    step(f"found adb at [cyan]{adb_path}[/]")
    return adb_path


def cmd_reconnect(args: argparse.Namespace) -> None:
    adb_path = require_adb()
    target = f"{args.host}:{args.port}"

    step("disconnecting existing sessions...")
    with console.status("[blue]Disconnecting..."):
        disconnect_result = adb(adb_path, "disconnect")
    console.print(f"[yellow]disconnect:[/] {disconnect_result}")

    step(f"connecting to {target}...")
    with console.status(f"[blue]Connecting to {target}..."):
        connect_result = adb(adb_path, "connect", target)

    if "connected to" in connect_result.lower():
        console.print(f"[bold green]✓ connect:[/] {connect_result}")
    else:
        console.print(f"[bold red]✗ connect:[/] {connect_result}")
        sys.exit(1)


def cmd_check(args: argparse.Namespace) -> None:
    step("searching for gradlew in local directory...")
    gradlew_path = find_gradlew()
    if not gradlew_path:
        console.print("[bold red]✗ Error:[/] no gradlew found in local directory.")
        sys.exit(1)
    step(f"found gradlew at [cyan]{gradlew_path}[/]")

    step("running ./gradlew assembleDebug (live output below)...")
    console.print(f"[blue]Using {gradlew_path}, running assembleDebug...[/]")
    returncode = run_live([str(gradlew_path), "assembleDebug"], cwd=gradlew_path.parent)

    if returncode == 0:
        console.print("[bold green]✓ Build succeeded.[/]")
    else:
        console.print(f"[bold red]✗ Build failed (exit {returncode}).[/]")
        sys.exit(returncode)


def cmd_devices(args: argparse.Namespace) -> None:
    adb_path = require_adb()
    step("querying `adb devices`...")
    output = adb(adb_path, "devices")
    devices = parse_devices(output.replace("List of devices attached", ""))

    if not devices:
        console.print("[yellow]No devices found.[/]")
        return

    step(f"found {len(devices)} device(s)")
    table = Table()
    table.add_column("Device ID")
    table.add_column("Status")
    for device in devices:
        status_style = "green" if device["status"] == "device" else "red"
        table.add_row(device["device_id"], f"[{status_style}]{device['status']}[/]")
    console.print(table)


def cmd_install(args: argparse.Namespace) -> None:
    adb_path = require_adb()
    apk_path = args.apk

    if not apk_path:
        step("no --apk given, searching build/outputs/apk/debug for apks...")
        candidates = sorted(Path.cwd().glob("**/build/outputs/apk/debug/*.apk"))
        if not candidates:
            console.print("[bold red]✗ Error:[/] no debug apk found, pass one with --apk.")
            sys.exit(1)
        apk_path = candidates[-1]
        step(f"using newest apk: [cyan]{apk_path}[/]")
    else:
        step(f"using given apk: [cyan]{apk_path}[/]")

    step("installing via `adb install -r` (live output below)...")
    console.print(f"[blue]Installing {apk_path}...[/]")
    returncode = run_live([str(adb_path), "install", "-r", str(apk_path)])
    if returncode == 0:
        console.print("[bold green]✓ Install succeeded.[/]")
    else:
        console.print(f"[bold red]✗ Install failed (exit {returncode}).[/]")
        sys.exit(returncode)


def cmd_deploy(args: argparse.Namespace) -> None:
    step("deploy: stage 1/2 - build")
    cmd_check(args)
    step("deploy: stage 2/2 - install")
    cmd_install(args)
    console.print("[bold green]✓ Deploy complete.[/]")


def cmd_logcat(args: argparse.Namespace) -> None:
    adb_path = require_adb()
    cmd = [str(adb_path), "logcat"]
    if args.filter:
        step(f"filtering logcat with [cyan]{args.filter}[/]")
        cmd.append(args.filter)
    step("streaming logcat (live output below, ctrl-C to stop)...")
    run_live(cmd)


def cmd_restart(args: argparse.Namespace) -> None:
    adb_path = require_adb()
    step("killing adb server...")
    console.print(adb(adb_path, "kill-server"))
    step("starting adb server...")
    console.print(adb(adb_path, "start-server"))
    console.print("[bold green]✓ adb server restarted.[/]")


def cmd_tcpip(args: argparse.Namespace) -> None:
    adb_path = require_adb()
    step(f"enabling tcpip mode on port {args.port}...")
    result = adb(adb_path, "tcpip", str(args.port))
    if "restarting in tcp" in result.lower():
        console.print(f"[bold green]✓ {result}[/]")
    else:
        console.print(f"[bold red]✗ {result}[/]")
        sys.exit(1)


def build_parser(repl: bool = False) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ADB with less pain", prog="" if repl else None)
    subparsers = parser.add_subparsers(dest="command", required=True)

    reconnect_parser = subparsers.add_parser(
        "reconnect", aliases=["rcon", "rconnect"],
        help="disconnect then reconnect to the Control Hub over wifi",
    )
    reconnect_parser.add_argument("--host", default=CONTROL_HUB_IP)
    reconnect_parser.add_argument("--port", default=CONTROL_HUB_PORT)
    reconnect_parser.set_defaults(func=cmd_reconnect)

    check_parser = subparsers.add_parser(
        "check", aliases=["compile", "checkcompile", "check-compile"],
        help="run ./gradlew assembleDebug with live output",
    )
    check_parser.set_defaults(func=cmd_check)

    devices_parser = subparsers.add_parser(
        "devices", aliases=["ls"], help="list connected/known devices",
    )
    devices_parser.set_defaults(func=cmd_devices)

    install_parser = subparsers.add_parser(
        "install", help="install the built debug apk onto the device",
    )
    install_parser.add_argument("--apk", type=Path, default=None, help="path to apk (default: newest debug build)")
    install_parser.set_defaults(func=cmd_install)

    deploy_parser = subparsers.add_parser(
        "deploy", aliases=["push"], help="build then install (check + install)",
    )
    deploy_parser.add_argument("--apk", type=Path, default=None)
    deploy_parser.set_defaults(func=cmd_deploy)

    logcat_parser = subparsers.add_parser(
        "logcat", help="stream device logcat live",
    )
    logcat_parser.add_argument("filter", nargs="?", default=None, help="optional logcat filter expression")
    logcat_parser.set_defaults(func=cmd_logcat)

    restart_parser = subparsers.add_parser(
        "restart", aliases=["restart-adb"], help="restart the adb server",
    )
    restart_parser.set_defaults(func=cmd_restart)

    tcpip_parser = subparsers.add_parser(
        "tcpip", aliases=["wireless"], help="enable tcpip mode on device over USB",
    )
    tcpip_parser.add_argument("--port", default=CONTROL_HUB_PORT)
    tcpip_parser.set_defaults(func=cmd_tcpip)

    return parser


def repl() -> None:
    console.print("[bold]fast-adb[/] repl. type a command, or 'exit'/'quit' to leave, 'help' for commands.")
    parser = build_parser(repl=True)

    while True:
        try:
            line = console.input("[bold cyan]fast-adb>[/] ").strip()
        except (EOFError, KeyboardInterrupt):
            console.print()
            break

        if not line:
            continue
        if line in ("exit", "quit", "q"):
            break
        if line in ("help", "?"):
            line = "-h"

        try:
            tokens = shlex.split(line)
        except ValueError as e:
            console.print(f"[bold red]✗ Error:[/] {e}")
            continue

        try:
            args = parser.parse_args(tokens)
        except SystemExit:
            continue

        try:
            args.func(args)
        except SystemExit:
            pass
        except KeyboardInterrupt:
            console.print("\n[yellow]interrupted.[/]")


def main():
    if len(sys.argv) == 1:
        repl()
        return

    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
