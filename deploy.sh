#!/bin/bash
# Offline deploy + diagnostics for 32008 slowstart.
#
# Run this while connected to the robot (USB or robot WiFi). It captures
# everything to deploy-log.txt so the log can be read afterwards, once the
# laptop is back on a network with internet.
#
#   bash deploy.sh
#
# Then rejoin your normal WiFi and say "done".

cd "$(dirname "$0")" || exit 1
ADB=/Users/georgehu/Library/Android/sdk/platform-tools/adb
LOG=/dev/null

{
  echo "===== 32008 deploy log ====="
  echo "date: $(date)"
  echo "cwd : $(pwd)"
  echo

  echo "===== network ====="
  /usr/sbin/networksetup -getairportnetwork en0 2>&1
  ifconfig 2>/dev/null | grep "inet " | grep -v 127.0.0.1
  echo

  echo "===== adb devices (USB) ====="
  "$ADB" devices -l 2>&1
  echo

  # If nothing on USB, try the two standard robot APs.
  if ! "$ADB" devices | grep -qE "device$|device "; then
    echo "===== no USB device, trying WiFi adb ====="
    for ip in 192.168.43.1 192.168.49.1; do
      echo "--- connect $ip:5555 ---"
      "$ADB" connect "$ip:5555" 2>&1
    done
    echo
    "$ADB" devices -l 2>&1
    echo
  fi

  echo "===== build ====="
  ./gradlew :TeamCode:assembleDebug 2>&1 | tail -25
  echo

  echo "===== install ====="
  APK=TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
  if [ -f "$APK" ]; then
    ls -lh "$APK"
    "$ADB" install -r "$APK" 2>&1 | tail -10
  else
    echo "APK NOT FOUND at $APK"
    find TeamCode/build/outputs -name "*.apk" 2>/dev/null | head
  fi
  echo

  echo "===== OpModes registered in the built source ====="
  grep -rho '@Autonomous(name *= *"[^"]*"' TeamCode/src/main/java 2>/dev/null | sed 's/.*name *= *"/  AUTO : /;s/"$//' | sort -u
  grep -rho '@TeleOp(name *= *"[^"]*"' TeamCode/src/main/java 2>/dev/null | sed 's/.*name *= *"/  TELEOP: /;s/"$//' | sort -u
  echo

  echo "===== recent robot log (crashes / hardware errors) ====="
  "$ADB" logcat -d -t 200 2>&1 | grep -iE "RobotCore|FATAL|AndroidRuntime|Exception|not found|configured" | tail -40

  echo
  echo "===== done ====="
} 2>&1 | tee "$LOG"

echo
echo "Wrote $(pwd)/$LOG"
echo "Rejoin your normal WiFi, then tell Claude 'done'."
