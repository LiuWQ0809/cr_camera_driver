#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PACKAGE_NAME="$(basename "$PROJECT_DIR")"

WORKSPACE_ROOT_DEFAULT="$PROJECT_DIR"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$WORKSPACE_ROOT_DEFAULT}"
BUILD_TYPE="Release"
PARALLEL_JOBS="$(nproc 2>/dev/null || echo 1)"
VERBOSE=false
CLEAN=false
ADDITIONAL_CMAKE_ARGS=()
COLCON_EXTRA_ARGS=()

log() {
    local level="$1"; shift
    printf "[%s] %s\n" "$level" "$*"
}

log_info() {
    log "INFO" "$@"
}

log_warn() {
    log "WARN" "$@"
}

log_err() {
    log "ERROR" "$@" >&2
}

usage() {
    cat <<EOF
Usage: $0 [options]

Options:
  -h, --help                 Show this help message
  --workspace-root PATH      Override the workspace root (default: $WORKSPACE_ROOT_DEFAULT)
  --build-type TYPE          CMake build type (Release, Debug, RelWithDebInfo, etc.)
  --jobs N                   Number of parallel workers (default: $PARALLEL_JOBS)
  --clean                    Remove build/install/log directories before building
  --verbose                  Enable more verbose colcon output
  --cmake-arg ARG            Pass ARG through to CMake (can be repeated)
  --colcon-arg ARG           Pass ARG through to colcon (can be repeated)
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --workspace-root)
            shift
            if [[ -z "${1:-}" ]]; then
                log_err "--workspace-root needs a path argument"
                exit 1
            fi
            WORKSPACE_ROOT="$1"
            shift
            ;;
        --build-type)
            shift
            if [[ -z "${1:-}" ]]; then
                log_err "--build-type needs a value"
                exit 1
            fi
            BUILD_TYPE="$1"
            shift
            ;;
        --jobs|--parallel-workers)
            opt="$1"
            shift
            if [[ -z "${1:-}" ]]; then
                log_err "$opt needs a value"
                exit 1
            fi
            PARALLEL_JOBS="$1"
            shift
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --cmake-arg)
            shift
            if [[ -z "${1:-}" ]]; then
                log_err "--cmake-arg needs a value"
                exit 1
            fi
            ADDITIONAL_CMAKE_ARGS+=("$1")
            shift
            ;;
        --colcon-arg)
            shift
            if [[ -z "${1:-}" ]]; then
                log_err "--colcon-arg needs a value"
                exit 1
            fi
            COLCON_EXTRA_ARGS+=("$1")
            shift
            ;;
        *)
            log_err "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

WORKSPACE_ROOT="$(realpath "$WORKSPACE_ROOT")"

if ! command -v colcon >/dev/null 2>&1; then
    log_err "colcon is not installed or not on PATH"
    exit 1
fi

if [[ "$CLEAN" = true ]]; then
    log_info "Cleaning previous build artifacts under $WORKSPACE_ROOT"
    rm -rf "$WORKSPACE_ROOT/build" "$WORKSPACE_ROOT/install" "$WORKSPACE_ROOT/log"
fi

log_info "Workspace root: $WORKSPACE_ROOT"
log_info "Package: $PACKAGE_NAME"
log_info "CMake build type: $BUILD_TYPE"
log_info "Parallel jobs: $PARALLEL_JOBS"

if [[ ! -d "$WORKSPACE_ROOT" ]]; then
    log_info "Creating workspace directory $WORKSPACE_ROOT"
    mkdir -p "$WORKSPACE_ROOT"
fi

cd "$WORKSPACE_ROOT"

if [[ ! -d "/usr/include/vpi" ]]; then
    log_warn "VPI headers not found at /usr/include/vpi (required for VIC conversion)"
fi

if ! ldconfig -p 2>/dev/null | grep -q 'nvvpi'; then
    log_warn "VPI runtime library 'nvvpi' not found in ldconfig cache"
fi

readonly CMAKE_ARGS_DEFAULT=("-DCMAKE_BUILD_TYPE=$BUILD_TYPE")
CMAKE_ARGS=("${CMAKE_ARGS_DEFAULT[@]}")
if [[ "${#ADDITIONAL_CMAKE_ARGS[@]}" -gt 0 ]]; then
    CMAKE_ARGS+=("${ADDITIONAL_CMAKE_ARGS[@]}")
fi

COLCON_CMD=(
    colcon
    build
    --base-path "$PROJECT_DIR"
    --packages-select "$PACKAGE_NAME"
    --parallel-workers "$PARALLEL_JOBS"
    --cmake-args "${CMAKE_ARGS[@]}"
)

if [[ "${#COLCON_EXTRA_ARGS[@]}" -gt 0 ]]; then
    COLCON_CMD+=("${COLCON_EXTRA_ARGS[@]}")
fi

if [[ "$VERBOSE" = true ]]; then
    log_info "Enabling verbose event handling for colcon"
    COLCON_CMD+=("--event-handlers" "console_direct+")
fi

log_info "Running: ${COLCON_CMD[*]}"

"${COLCON_CMD[@]}"
