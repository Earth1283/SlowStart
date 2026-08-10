#!/bin/bash
# Reopen the FTC project in Android Studio instead of IntelliJ IDEA.
#
# WHY: IntelliJ IDEA has no Android application plugin -- only Gradle-DSL syntax
# fragments. It cannot build an "Android App" run configuration, which is why
# TeamCode shows a red X in Run/Debug Configurations, the Run button is gray, and
# no device is ever detected. Android Studio has the real plugin.
#
#   bash open-in-android-studio.sh
#
# The IntelliJ .idea has already been backed up to idea-backup-intellij-*.tar.gz.

set -e
cd "$(dirname "$0")"
PROJ="$(pwd)"

echo "Project: $PROJ"
echo

# 1. IntelliJ must be closed first -- it rewrites .idea on exit and would undo this.
if pgrep -f "IntelliJ IDEA.app/Contents/MacOS/idea" >/dev/null 2>&1; then
  echo "IntelliJ IDEA is still running."
  echo "Quit it (Cmd-Q, save anything you care about), then run this script again."
  exit 1
fi
echo "[1/4] IntelliJ IDEA is closed."

# 2. Drop the IntelliJ-written IDE config so Android Studio generates its own.
#    Backed up already; .idea is fully regenerable from the Gradle build.
if [ -d .idea ]; then
  rm -rf .idea
  echo "[2/4] Removed .idea (IntelliJ's). Android Studio will regenerate it."
else
  echo "[2/4] No .idea present."
fi

# 3. Drop IDE-local Gradle state. NOT the wrapper, NOT any build file.
rm -rf .gradle
echo "[3/4] Cleared .gradle IDE cache (wrapper and build files untouched)."

# 4. Hand it to Android Studio.
open -a "Android Studio" "$PROJ"
echo "[4/4] Opening in Android Studio..."

echo
echo "Next, inside Android Studio:"
echo "  - let the Gradle sync finish (progress bar, bottom right)"
echo "  - the run configuration 'TeamCode' should appear with NO red X"
echo "  - plug the Control Hub in by USB; it appears in the device dropdown"
echo "  - Run turns green"
