#!/usr/bin/env bash
#
# Link this repo's TeamCode Java sources into the FTC SDK clone.
#
# The FTC SDK is cloned outside version control and its TeamCode module is what
# gradle actually compiles. We never copy files into it -- we symlink, so editing
# a file in this repo is picked up by the next build with no sync step to forget,
# and this repo stays the single source of truth.
#
# This is called by BOTH install.sh (as a build step) and build.sh (before every
# gradle run). That second call is the important one: a new .java file is not
# visible to gradle until it has been linked, and the failure it produces -- some
# other class "cannot find symbol" -- points at the wrong file entirely. Linking
# before every build makes that trap impossible to hit.
#
#   ./link-teamcode.sh              link, and mention what changed
#   ./link-teamcode.sh --quiet      link, print only on error
#
# Environment override: FTC_SDK_DIR (also read from .ftc-env.sh by build.sh).

set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Where the OpModes live, relative to both this repo and the SDK clone. The two
# paths are the same by design -- the package structure is what makes the
# symlinks resolve.
TEAMCODE_PKG="TeamCode/src/main/java/org/firstinspires/ftc/teamcode"

FTC_SDK_DIR="${FTC_SDK_DIR:-$REPO_ROOT/ftc-sdk}"

QUIET=0
if [[ "${1:-}" == "--quiet" ]]; then
    QUIET=1
elif [[ $# -gt 0 ]]; then
    echo "error: unknown argument: $1" >&2
    exit 2
fi

src="$REPO_ROOT/$TEAMCODE_PKG"
dst="$FTC_SDK_DIR/$TEAMCODE_PKG"

[[ -d "$src" ]] || { echo "error: no sources at $src" >&2; exit 1; }
[[ -d "$FTC_SDK_DIR" ]] || { echo "error: FTC SDK not at $FTC_SDK_DIR -- run ./install.sh first" >&2; exit 1; }

mkdir -p "$dst"

# Link every .java file. -f replaces an existing link, -n treats a link to a
# directory as a link rather than following it.
linked=0
for f in "$src"/*.java; do
    [[ -e "$f" ]] || continue
    ln -sfn "$f" "$dst/$(basename "$f")"
    linked=$((linked + 1))
done

# Remove links left behind by a file this repo has renamed or deleted. Without
# this a stale symlink becomes a dangling path, and the build fails on a file
# nobody can find in git.
removed=0
for l in "$dst"/*.java; do
    [[ -L "$l" ]] || continue
    target="$(readlink "$l")"
    if [[ "$target" == "$REPO_ROOT"/* && ! -e "$target" ]]; then
        rm -f "$l"
        removed=$((removed + 1))
    fi
done

if (( ! QUIET )); then
    msg="linked $linked file(s) into $dst"
    (( removed > 0 )) && msg+=" (removed $removed stale)"
    echo "$msg"
fi
