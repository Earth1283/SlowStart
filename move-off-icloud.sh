#!/bin/bash
# Move the FTC project OFF iCloud Drive.
#
# WHY: ~/Desktop is iCloud-synced ("Desktop & Documents Folders"). iCloud's file
# provider intercepts files as Gradle writes them, and reads then fail with
#     java.io.IOException: Operation timed out
# which is what every one of those seven Gradle stack traces bottoms out at.
#
# It is also what caused today's directory chaos: the slowstart / "SlowStart 2"
# duplication is an iCloud sync conflict on a case-insensitive filesystem, and the
# gutted folders and vanished .git were the same thing.
#
# A Gradle build directory must never live in iCloud Drive. This moves the project
# to ~/FTCDev/Slowstart, which iCloud does not touch.
#
#   bash move-off-icloud.sh

set -e
SRC="/Users/georgehu/Desktop/Slowstart"
DEST_PARENT="/Users/georgehu/FTCDev"
DEST="$DEST_PARENT/Slowstart"

# 1. No IDE may hold the tree open during the move.
if pgrep -f "Android Studio.app/Contents/MacOS/studio" >/dev/null 2>&1 \
   || pgrep -f "IntelliJ IDEA.app/Contents/MacOS/idea" >/dev/null 2>&1; then
  echo "Android Studio (or IntelliJ) is still running."
  echo "Quit it completely with Cmd-Q, then run this script again."
  exit 1
fi
echo "[1/6] No IDE running."

if [ -e "$DEST" ]; then
  echo "ERROR: $DEST already exists. Move or rename it first; refusing to overwrite."
  exit 1
fi
mkdir -p "$DEST_PARENT"
echo "[2/6] Target ready: $DEST"

# 2. Copy source + config only. Build output and IDE caches are regenerable, and
#    they are the bulk of the size AND the files iCloud is actively fighting.
echo "[3/6] Copying source (excluding build output and caches)..."
rsync -a \
  --exclude 'build/' \
  --exclude '.gradle/' \
  --exclude '.idea/' \
  --exclude '*.tar.gz' \
  "$SRC/" "$DEST/"

# 3. Sanity check that the things that matter actually arrived.
echo "[4/6] Verifying..."
MISSING=0
for f in \
  "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/BlueCloseAuto.java" \
  "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/BlueFarAuto.java" \
  "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/AutoAimSubsystem.java" \
  "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/v2/AASSTEST.java" \
  "team-config.yaml" "settings.gradle" "local.properties" "gradlew"; do
  if [ -f "$DEST/$f" ]; then echo "    ok  $f"; else echo "    MISSING  $f"; MISSING=1; fi
done
[ "$MISSING" -eq 0 ] || { echo "Verification FAILED. Original untouched at $SRC."; exit 1; }

# 4. Prove it builds at the new location before anyone trusts it.
echo "[5/6] Building at the new location (this is the real test)..."
cd "$DEST"
chmod +x gradlew
./gradlew :TeamCode:assembleDebug 2>&1 | tail -6

# 5. The original is LEFT IN PLACE on purpose. Nothing is deleted here.
echo
echo "[6/6] Done."
echo
echo "  NEW working folder : $DEST"
echo "  Old copy left at   : $SRC   (still on iCloud -- delete it once you are happy)"
echo
echo "In Android Studio: File > Open > $DEST"
echo "Close the old project. Do not open the Desktop copy again."
echo
echo "Also worth doing: System Settings > Apple Account > iCloud > Drive >"
echo "  turn OFF 'Desktop & Documents Folders', or keep all code out of those two folders."
