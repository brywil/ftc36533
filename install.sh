#!/usr/bin/env bash
#
# ftc36533 -- bring a bare machine to the point where this code builds and runs.
#
# Assumes NOTHING is installed. Installs, in order: system packages, a JDK the
# Android Gradle Plugin will accept, the Android SDK command-line tools, the FTC
# SDK itself, this repo's OpModes linked into it, and a Python venv for the
# offline vision harness. Then it proves the result by building the APK and
# running the vision self-test.
#
#   ./install.sh                 full install, then verify
#   ./install.sh --dry-run       print every command, change nothing
#   ./install.sh --vision-only   just the Python side (no sudo, no Android SDK)
#   ./install.sh --no-build      install, skip the gradle build at the end
#   ./install.sh -h              all flags
#
# Re-running is safe: every step checks for its own result first.

set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The FTC SDK tag to build against. Pinned rather than "latest" so a season
# rollover cannot silently change what the team is building, and so two laptops
# set up a month apart get the same thing.
FTC_SDK_TAG="${FTC_SDK_TAG:-v12.0}"
FTC_SDK_REPO="${FTC_SDK_REPO:-https://github.com/FIRST-Tech-Challenge/FtcRobotController}"
# NOT a dot-directory. Gradle 9 refuses to configure a root project whose
# directory name starts with '.', and the error it gives ("project name must not
# start or end with a '.'") names the project, not the path you chose.
FTC_SDK_DIR="${FTC_SDK_DIR:-$REPO_ROOT/ftc-sdk}"

ANDROID_SDK_ROOT="${ANDROID_SDK_ROOT:-$HOME/Android/Sdk}"

# Resolved from Google's own package index at run time; this is only the fallback
# for when that index cannot be reached.
CMDLINE_TOOLS_FALLBACK="16111833"

# AGP 8.13 (what FTC SDK 12.0 pins) needs JDK 17 or newer.
MIN_JDK=17

ENV_FILE="$REPO_ROOT/.ftc-env.sh"
TEAMCODE_PKG="TeamCode/src/main/java/org/firstinspires/ftc/teamcode"

DRY_RUN=0
VISION_ONLY=0
DO_BUILD=1
SKIP_PACKAGES=0

# ---------------------------------------------------------------- plumbing

RED=''; GRN=''; YLW=''; BLD=''; RST=''
if [[ -t 1 ]]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YLW=$'\033[33m'; BLD=$'\033[1m'; RST=$'\033[0m'
fi

step() { printf '\n%s==> %s%s\n' "$BLD" "$*" "$RST"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '%s    warning: %s%s\n' "$YLW" "$*" "$RST" >&2; }
die()  { printf '%s\nerror: %s%s\n' "$RED" "$*" "$RST" >&2; exit 1; }
ok()   { printf '%s    ok: %s%s\n' "$GRN" "$*" "$RST"; }

# Every mutating command goes through this, so --dry-run is honest rather than a
# claim. Anything that only READS state is called directly.
run() {
    if (( DRY_RUN )); then
        printf '    [dry-run] %s\n' "$*"
        return 0
    fi
    "$@"
}

trap 'die "failed at line $LINENO: $BASH_COMMAND"' ERR

usage() {
    sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

Flags:
  --dry-run            print what would happen; change nothing
  --vision-only        Python venv + self-test only; no sudo, no Android SDK
  --skip-packages      assume the system packages are already there (no sudo)
  --no-build           skip the gradle APK build at the end
  --sdk-tag TAG        FTC SDK tag to build against (default: v12.0)
  --android-root DIR   where to put the Android SDK (default: ~/Android/Sdk)
  -h, --help           this

Environment overrides: FTC_SDK_TAG, FTC_SDK_REPO, FTC_SDK_DIR, ANDROID_SDK_ROOT
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)      DRY_RUN=1 ;;
        --vision-only)  VISION_ONLY=1 ;;
        --skip-packages) SKIP_PACKAGES=1 ;;
        --no-build)     DO_BUILD=0 ;;
        --sdk-tag)      FTC_SDK_TAG="${2:?--sdk-tag needs a value}"; shift ;;
        --android-root) ANDROID_SDK_ROOT="${2:?--android-root needs a value}"; shift ;;
        -h|--help)      usage; exit 0 ;;
        *)              usage >&2; die "unknown argument: $1" ;;
    esac
    shift
done

SUDO=""
need_sudo() {
    [[ $EUID -eq 0 ]] && return 0
    command -v sudo >/dev/null 2>&1 || die "not root and sudo is not installed"
    SUDO="sudo"
}

# ---------------------------------------------------------------- packages

PKGS_APT=(git curl unzip ca-certificates python3 python3-venv python3-pip
          "openjdk-${MIN_JDK}-jdk-headless")

install_packages() {
    step "System packages"

    if ! command -v apt-get >/dev/null 2>&1; then
        # Deliberately not guessing at another package manager's names. An
        # installer that half-works on an untested distro is worse than one that
        # tells you exactly what it needs.
        warn "no apt-get here -- this installer only automates Debian/Ubuntu."
        info "Install the equivalents by hand, then re-run with --vision-only"
        info "for the Python side:"
        info "    ${PKGS_APT[*]}"
        die "unsupported package manager"
    fi

    local missing=()
    local p
    for p in "${PKGS_APT[@]}"; do
        dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
    done

    if [[ ${#missing[@]} -eq 0 ]]; then
        ok "all present: ${PKGS_APT[*]}"
        return 0
    fi

    info "installing: ${missing[*]}"
    need_sudo
    run $SUDO apt-get update -qq
    run $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "${missing[@]}"
    ok "installed ${#missing[@]} package(s)"
}

# ---------------------------------------------------------------- jdk

# Returns the feature version (17, 21, ...) of a JDK home, or nothing.
jdk_version_of() {
    local home="$1" v=""
    [[ -x "$home/bin/javac" ]] || return 1
    if [[ -r "$home/release" ]]; then
        v="$(sed -n 's/^JAVA_VERSION="\([0-9][0-9]*\).*/\1/p' "$home/release" | head -1)"
    fi
    [[ -z "$v" ]] && v="$("$home/bin/javac" -version 2>&1 | sed -n 's/^javac \([0-9][0-9]*\).*/\1/p')"
    [[ -n "$v" ]] && printf '%s' "$v"
}

JAVA_HOME_RESOLVED=""

pick_jdk() {
    step "JDK (AGP 8.13 needs ${MIN_JDK}+)"

    local best="" best_v=0 cand v
    local -a candidates=()

    [[ -n "${JAVA_HOME:-}" ]] && candidates+=("$JAVA_HOME")
    if command -v javac >/dev/null 2>&1; then
        candidates+=("$(dirname "$(dirname "$(readlink -f "$(command -v javac)")")")")
    fi
    # Nullglob so an empty /usr/lib/jvm does not leave a literal glob in the list.
    shopt -s nullglob
    candidates+=(/usr/lib/jvm/*)
    shopt -u nullglob

    for cand in "${candidates[@]}"; do
        v="$(jdk_version_of "$cand" 2>/dev/null || true)"
        [[ -z "$v" ]] && continue
        (( v < MIN_JDK )) && continue
        # Prefer the lowest version that still qualifies: AGP is tested against
        # its minimum far more than against whatever the distro ships next year.
        if [[ -z "$best" ]] || (( v < best_v )); then
            best="$cand"; best_v="$v"
        fi
    done

    [[ -z "$best" ]] && die "no JDK >= $MIN_JDK found after install -- check 'apt policy openjdk-${MIN_JDK}-jdk-headless'"

    JAVA_HOME_RESOLVED="$best"
    export JAVA_HOME="$best"
    export PATH="$JAVA_HOME/bin:$PATH"
    ok "JDK $best_v at $best"
}

# ---------------------------------------------------------------- android sdk

newest_cmdline_tools() {
    # Ask Google's package index which build is current instead of pinning a build
    # id that goes stale. Falls back to a known-good one if the index is
    # unreachable, because a hung installer is worse than a slightly old zip.
    local id
    id="$(curl -sS --max-time 30 https://dl.google.com/android/repository/repository2-3.xml 2>/dev/null \
          | grep -oE 'commandlinetools-linux-[0-9]+_latest\.zip' \
          | sed -E 's/.*-([0-9]+)_latest\.zip/\1/' \
          | sort -n | tail -1)" || true
    printf '%s' "${id:-$CMDLINE_TOOLS_FALLBACK}"
}

install_android_sdk() {
    step "Android SDK command-line tools"

    local sdkmanager="$ANDROID_SDK_ROOT/cmdline-tools/latest/bin/sdkmanager"

    if [[ -x "$sdkmanager" ]]; then
        ok "already at $ANDROID_SDK_ROOT"
    else
        local id zip tmp
        id="$(newest_cmdline_tools)"
        zip="commandlinetools-linux-${id}_latest.zip"
        info "fetching $zip"

        tmp="$(mktemp -d)"
        run curl -fSL --retry 3 -o "$tmp/$zip" \
            "https://dl.google.com/android/repository/$zip"
        run mkdir -p "$ANDROID_SDK_ROOT/cmdline-tools"
        run unzip -q -o "$tmp/$zip" -d "$tmp"
        # The zip unpacks to cmdline-tools/ but sdkmanager insists on living in a
        # version directory -- "latest" is the conventional one. Without this move
        # every sdkmanager call fails with a bare, unhelpful path error.
        run rm -rf "$ANDROID_SDK_ROOT/cmdline-tools/latest"
        run mv "$tmp/cmdline-tools" "$ANDROID_SDK_ROOT/cmdline-tools/latest"
        run rm -rf "$tmp"
        ok "installed to $ANDROID_SDK_ROOT"
    fi

    export ANDROID_SDK_ROOT ANDROID_HOME="$ANDROID_SDK_ROOT"

    local compile_sdk="${1:-30}"

    if (( DRY_RUN )); then
        info "[dry-run] would accept licenses, then install platform-tools and platforms;android-$compile_sdk"
        return 0
    fi

    step "Android SDK licenses and packages"
    # Unaccepted licenses do not fail loudly at this stage -- they fail later,
    # inside the gradle build, as a package-not-found that reads like a network
    # problem. Accept them here where the cause is obvious.
    yes 2>/dev/null | "$sdkmanager" --sdk_root="$ANDROID_SDK_ROOT" --licenses >/dev/null || true

    info "platform-tools, platforms;android-$compile_sdk"
    "$sdkmanager" --sdk_root="$ANDROID_SDK_ROOT" \
        "platform-tools" "platforms;android-$compile_sdk" >/dev/null
    # Build-tools is deliberately NOT pinned here. AGP knows which version it
    # wants and downloads it itself once the licenses above are accepted; pinning
    # a guess just produces a second, unused copy.
    ok "Android SDK ready"
}

# ---------------------------------------------------------------- ftc sdk

clone_ftc_sdk() {
    step "FTC SDK $FTC_SDK_TAG"

    if [[ -d "$FTC_SDK_DIR/.git" ]]; then
        local have
        have="$(git -C "$FTC_SDK_DIR" describe --tags --exact-match 2>/dev/null || echo "?")"
        if [[ "$have" == "$FTC_SDK_TAG" ]]; then
            ok "already at $FTC_SDK_TAG"
            return 0
        fi
        info "clone is at $have, moving to $FTC_SDK_TAG"
        run git -C "$FTC_SDK_DIR" fetch --depth 1 origin "refs/tags/$FTC_SDK_TAG:refs/tags/$FTC_SDK_TAG"
        run git -C "$FTC_SDK_DIR" checkout -q "$FTC_SDK_TAG"
    else
        run git clone --depth 1 --branch "$FTC_SDK_TAG" -q "$FTC_SDK_REPO" "$FTC_SDK_DIR"
    fi
    ok "$FTC_SDK_DIR"
}

# Read compileSdkVersion out of the SDK we actually cloned rather than hardcoding
# it. FIRST bumps this between seasons and a stale constant here would install the
# wrong platform and fail in gradle.
detect_compile_sdk() {
    local f="$FTC_SDK_DIR/build.common.gradle" v=""
    [[ -r "$f" ]] && v="$(sed -n 's/^[[:space:]]*compileSdkVersion[[:space:]]*\([0-9][0-9]*\).*/\1/p' "$f" | head -1)"
    printf '%s' "${v:-30}"
}

link_teamcode() {
    step "Linking this repo's OpModes into the FTC SDK"

    local src="$REPO_ROOT/$TEAMCODE_PKG"
    local dst="$FTC_SDK_DIR/$TEAMCODE_PKG"
    run mkdir -p "$dst"

    # Symlinks, not copies: edit a file here and the next build picks it up, with
    # no sync step to forget. The repo stays the single source of truth.
    local f n linked=0
    for f in "$src"/*.java; do
        [[ -e "$f" ]] || continue
        n="$(basename "$f")"
        run ln -sfn "$f" "$dst/$n"
        linked=$((linked + 1))
    done

    # Drop links left behind by a file this repo has since renamed or deleted;
    # otherwise a stale symlink becomes a dangling path and the build fails on a
    # file nobody can find in git.
    local l target
    for l in "$dst"/*.java; do
        [[ -L "$l" ]] || continue
        target="$(readlink "$l")"
        if [[ "$target" == "$REPO_ROOT"/* && ! -e "$target" ]]; then
            run rm -f "$l"
        fi
    done

    ok "$linked file(s) linked into $dst"
}

write_env_file() {
    step "Writing $ENV_FILE"
    if (( DRY_RUN )); then
        info "[dry-run] would write JAVA_HOME / ANDROID_SDK_ROOT exports"
        return 0
    fi
    cat > "$ENV_FILE" <<EOF
# Generated by install.sh -- source this before running gradle by hand.
# Regenerate by re-running ./install.sh; do not edit.
export JAVA_HOME="$JAVA_HOME_RESOLVED"
export ANDROID_SDK_ROOT="$ANDROID_SDK_ROOT"
export ANDROID_HOME="$ANDROID_SDK_ROOT"
export PATH="\$JAVA_HOME/bin:\$ANDROID_SDK_ROOT/platform-tools:\$PATH"
export FTC_SDK_DIR="$FTC_SDK_DIR"
EOF
    # Gradle finds the Android SDK through local.properties without any
    # environment at all, which is what makes the build work from an IDE too.
    printf 'sdk.dir=%s\n' "$ANDROID_SDK_ROOT" > "$FTC_SDK_DIR/local.properties"
    ok "wrote $ENV_FILE and $FTC_SDK_DIR/local.properties"
}

# ---------------------------------------------------------------- python

setup_venv() {
    step "Python venv for the vision harness"

    local venv="$REPO_ROOT/.venv"
    if [[ ! -x "$venv/bin/python" ]]; then
        run python3 -m venv "$venv"
    fi
    run "$venv/bin/pip" install -q --upgrade pip
    run "$venv/bin/pip" install -q -r "$REPO_ROOT/tools/requirements.txt"
    ok "$venv"
}

# ---------------------------------------------------------------- verify

verify() {
    step "Verifying"

    if (( DRY_RUN )); then
        info "[dry-run] would run the vision self-test and build the APK"
        return 0
    fi

    info "vision self-test"
    ( cd "$REPO_ROOT/tools" && "$REPO_ROOT/.venv/bin/python" selftest.py >/dev/null )
    ok "vision self-test passed"

    if (( VISION_ONLY )) || (( ! DO_BUILD )); then
        info "skipping the gradle build"
        return 0
    fi

    info "gradle :TeamCode:assembleDebug (first run downloads gradle itself; slow)"
    ( cd "$FTC_SDK_DIR" && JAVA_HOME="$JAVA_HOME_RESOLVED" ./gradlew --no-daemon -q :TeamCode:assembleDebug )

    # Check the artifact, not the exit status. A build can report success with
    # nothing to show for it if the module name is wrong.
    local apk
    apk="$(find "$FTC_SDK_DIR/TeamCode/build/outputs/apk" -name '*.apk' 2>/dev/null | head -1 || true)"
    [[ -n "$apk" ]] || die "gradle exited 0 but produced no APK"
    ok "built $apk"
}

# ---------------------------------------------------------------- main

main() {
    printf '%sftc36533 installer%s  (repo: %s)\n' "$BLD" "$RST" "$REPO_ROOT"
    (( DRY_RUN )) && info "DRY RUN -- nothing will be changed"

    if (( VISION_ONLY )); then
        command -v python3 >/dev/null 2>&1 || die "python3 not installed; drop --vision-only"
        setup_venv
        verify
    else
        if (( SKIP_PACKAGES )); then
            info "skipping system packages at your request"
        else
            install_packages
        fi
        pick_jdk
        clone_ftc_sdk
        install_android_sdk "$(detect_compile_sdk)"
        link_teamcode
        write_env_file
        setup_venv
        verify
    fi

    step "Done"
    cat <<EOF
    Build the APK:        ./build.sh
    Install to a hub:     ./build.sh install
    Run the vision test:  cd tools && ../.venv/bin/python selftest.py
    Gradle by hand:       source .ftc-env.sh && cd ftc-sdk && ./gradlew tasks
EOF
}

main "$@"
