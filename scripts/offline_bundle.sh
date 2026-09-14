#!/usr/bin/env bash
#
# offline_bundle.sh — move this repo to/from an offline machine (e.g. a Radxa with
# no internet) via a git bundle: one self-contained, integrity-checked file that
# carries full history and travels over a USB stick or LAN scp.
#
# On the machine WITH the up-to-date repo (and internet):
#     scripts/offline_bundle.sh create                 # -> ~/diffsim_hardware.bundle
#     scripts/offline_bundle.sh create -o /mnt/usb/x.bundle --all
#     scripts/offline_bundle.sh create --send radxa@radxa-cubie-a7z:~   # also scp it
#
# Copy the .bundle across (USB or scp), then on the OFFLINE machine:
#     scripts/offline_bundle.sh apply ~/diffsim_hardware.bundle         # pull into an
#                                                                       # existing clone
#     scripts/offline_bundle.sh apply ~/x.bundle --clone ~/diffsim_hardware  # fresh clone
#
# After applying, run `make clean` before rebuilding: git doesn't preserve mtimes,
# so stale objects can otherwise linger and cause link errors.

set -euo pipefail

die() { echo "error: $*" >&2; exit 1; }

usage() {
    sed -n '3,26p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

cmd="${1:-}"; shift || true

case "$cmd" in
create)
    out="$HOME/diffsim_hardware.bundle"
    ref=""            # empty -> current branch
    send=""
    while [ $# -gt 0 ]; do
        case "$1" in
            -o|--output) out="${2:?-o needs a path}"; shift 2 ;;
            --all)       ref="--all"; shift ;;
            -b|--branch) ref="${2:?-b needs a branch}"; shift 2 ;;
            --send)      send="${2:?--send needs a destination}"; shift 2 ;;
            -h|--help)   usage 0 ;;
            *) die "unknown option: $1 (see --help)" ;;
        esac
    done

    root="$(git rev-parse --show-toplevel 2>/dev/null)" || die "not inside a git repo"
    if [ -z "$ref" ]; then
        ref="$(git -C "$root" rev-parse --abbrev-ref HEAD)"
        [ "$ref" != "HEAD" ] || die "detached HEAD; pass -b <branch> or --all"
    fi

    echo "bundling '$ref' from $root -> $out"
    if [ "$ref" = "--all" ]; then
        git -C "$root" bundle create "$out" --all
    else
        # include HEAD so `git clone` from the bundle knows what to check out
        git -C "$root" bundle create "$out" HEAD "$ref"
    fi
    git -C "$root" bundle verify "$out" >/dev/null && echo "verified ok"
    echo "size: $(du -h "$out" | cut -f1)"

    if [ -n "$send" ]; then
        echo "copying to $send ..."
        scp "$out" "$send"
    fi
    echo
    echo "next, on the offline machine:"
    echo "    scripts/offline_bundle.sh apply <path-to-$(basename "$out")>"
    ;;

apply)
    [ $# -ge 1 ] || die "apply needs a bundle path (see --help)"
    bundle="$1"; shift
    ref="main"
    clone_dir=""
    while [ $# -gt 0 ]; do
        case "$1" in
            -b|--branch) ref="${2:?-b needs a branch}"; shift 2 ;;
            --clone)     clone_dir="${2:?--clone needs a target dir}"; shift 2 ;;
            -h|--help)   usage 0 ;;
            *) die "unknown option: $1 (see --help)" ;;
        esac
    done

    [ -f "$bundle" ] || die "no such bundle: $bundle"
    git bundle verify "$bundle" >/dev/null || die "bundle failed verification (corrupt copy?)"

    if [ -n "$clone_dir" ]; then
        echo "cloning '$ref' from $bundle -> $clone_dir"
        git clone -b "$ref" "$bundle" "$clone_dir"
        echo "done. cd $clone_dir && make clean && make ..."
    else
        git rev-parse --show-toplevel >/dev/null 2>&1 || \
            die "not inside a git repo; use --clone <dir> for a fresh checkout"
        echo "pulling '$ref' from $bundle into $(git rev-parse --show-toplevel)"
        git pull "$bundle" "$ref"
        echo "done. run: make clean && make ..."
    fi
    ;;

-h|--help|"") usage 0 ;;
*) die "unknown command: $cmd (expected 'create' or 'apply'; see --help)" ;;
esac
