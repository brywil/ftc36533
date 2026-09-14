#!/usr/bin/env bash
#
# Build (and optionally install) the TeamCode APK.
#
#   ./build.sh              assembleDebug -- the APK you side-load
#   ./build.sh install      assembleDebug + push to a connected Control Hub / phone
#   ./build.sh clean        wipe build outputs
#   ./build.sh <task...>    any other gradle task, passed straight through
#
# Requires ./install.sh to have run first -- it generates .ftc-env.sh.

set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$REPO_ROOT/.ftc-env.sh"

if [[ ! -r "$ENV_FILE" ]]; then
    echo "error: $ENV_FILE is missing -- run ./install.sh first" >&2
    exit 1
fi

# shellcheck source=/dev/null
source "$ENV_FILE"

[[ -d "$FTC_SDK_DIR" ]] || { echo "error: FTC SDK not at $FTC_SDK_DIR -- re-run ./install.sh" >&2; exit 1; }

cd "$FTC_SDK_DIR"

case "${1:-}" in
    "")
        ./gradlew :TeamCode:assembleDebug
        # Report the artifact rather than the exit status: gradle can succeed and
        # leave nothing behind if the module name is wrong.
        apk="$(find TeamCode/build/outputs/apk -name '*.apk' 2>/dev/null | head -1 || true)"
        [[ -n "$apk" ]] || { echo "error: build succeeded but produced no APK" >&2; exit 1; }
        echo "APK: $FTC_SDK_DIR/$apk"
        ;;
    install)
        # installDebug needs adb to see the hub. Over Wi-Fi that means
        # 'adb connect 192.168.43.1:5555' first.
        ./gradlew :TeamCode:installDebug
        ;;
    clean)
        ./gradlew clean
        ;;
    *)
        ./gradlew "$@"
        ;;
esac
