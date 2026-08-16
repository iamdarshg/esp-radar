# Reusable ESP32 build environment for this project (Windows / Git Bash).
#
# Sources: the manually-installed Espressif IDF v5.4.4 toolchain. Every
# workaround below was diagnosed from the `esp-idf-sys` / `embuild` build-script
# source; see README "Building" for the full story.
#
# Usage:  source firmware/esp-env.sh [logfile]
#   - defines `esp_cargo` to run `cargo <args>` with this environment,
#     appending output to the given logfile (default: check.log).
#   - `esp_clean` removes stale logs first.
#
# Workarounds encoded here:
#   1. RUSTUP_TOOLCHAIN=esp  — the esp nightly toolchain (has -Zbuild-std).
#   2. IDF_PYTHON_ENV_PATH  — the IDF venv lives at C:/Espressif/python_env
#      directly (not .../idf5.4_py3.12_env). Honoured by idf_tools.py:1744.
#   3. venv Scripts dir FIRST on PATH — embuild (espidf.rs:460) resolves
#      `python` via `which("python")` and passes it as -DPYTHON to CMake.
#      The venv's python must win over the system Python 3.12.
#   4. LIBCLANG_PATH = the *directory* holding libclang.dll — clang-sys
#      appends the platform libclang filename to this value (build/common.rs).
#      It is NOT the full path to the .dll.
#   5. esp-clang bin dir FIRST on PATH — clang-sys 1.9.1 loads libclang.dll
#      via libloading (`LoadLibraryExW` flags=0), so Windows resolves
#      libclang.dll's OWN runtime deps (libclang-cpp.dll, libLLVM-20.dll,
#      libc++.dll, libstdc++-6.dll, ...) from the app dir, cwd, system dirs
#      and PATH — NOT from the DLL's own directory. Putting the bin dir on
#      PATH makes those deps resolvable from the cargo build cwd.
#   6. ESP_IDF_SDKCONFIG_DEFAULTS — esp-idf-sys's native build only consults
#      sdkconfig.defaults from env/metadata, NOT the firmware dir. Without it
#      the generated sdkconfig loses CONFIG_HTTPD_WS_SUPPORT=y (WebSocket
#      dashboard, radar_web::server) and the OTA partition table. Set it to the
#      current firmware dir's file (we're sourced from inside firmware/*).
#   7. Single-core builds — the native ESP-IDF CMake/ninja build fans out to
#      every core by default (hundreds of clang instances → multi-GB RAM). The
#      `cmake` crate only passes `--parallel <N>` to `cmake --build` when
#      $NUM_JOBS is set, and ninja otherwise uses all cores. Cap it here, and
#      cap rustc to one job too. (Bump NUM_JOBS/CARGO_BUILD_JOBS for speed.)

export IDF_PATH="$IDF_PATH_VAL"
export IDF_PYTHON_ENV_PATH="$PYENV"
export RUSTUP_TOOLCHAIN=esp
export LIBCLANG_PATH="$CLANG_BIN"

# esp-idf-sys reads $ESP_IDF_SDKCONFIG_DEFAULTS (a list of files, whitespace-
# separated) and passes them to idf.py's SDKCONFIG_DEFAULTS. Source this script
# from inside a firmware/* dir so pwd is that dir's root.
if [ -f "sdkconfig.defaults" ]; then
    export ESP_IDF_SDKCONFIG_DEFAULTS="$(pwd -W 2>/dev/null || pwd)/sdkconfig.defaults"
fi

# Baselines (match the installed layout).
ESP_BASE="/c/Espressif"
IDF_PATH_VAL="$ESP_BASE/frameworks/esp-idf-v5.4"
TOOLS_BASE="$ESP_BASE/tools"
PYENV="$ESP_BASE/python_env"
RUSTUP_BASE="/c/Users/Darsh Gupta/.rustup"
CLANG_BIN="$RUSTUP_BASE/toolchains/esp/xtensa-esp32-elf-clang/esp-clang/bin"

export IDF_PATH="$IDF_PATH_VAL"
export IDF_PYTHON_ENV_PATH="$PYENV"
export RUSTUP_TOOLCHAIN=esp
export LIBCLANG_PATH="$CLANG_BIN"

export PATH="$PYENV/Scripts:$CLANG_BIN:/c/Users/Darsh Gupta/.cargo/bin:$RUSTUP_BASE/toolchains/esp/bin:$TOOLS_BASE/xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin:$TOOLS_BASE/cmake/3.30.2/bin:$TOOLS_BASE/ninja/1.12.1:$IDF_PATH_VAL/tools:$PATH"

# Workaround 7: cap the native build (ninja via `cmake --build --parallel`)
# and rustc to a single job to keep RAM flat.
export NUM_JOBS=1
export CARGO_BUILD_JOBS=1

# Convenience wrappers. Run from a firmware/* directory so cargo discovers that
# directory's .cargo/config.toml (xtensa target, C:/rt target-dir, build-std).
#
# Usage: esp_cargo [logfile] -- <cargo args...>   (default logfile: check.log)
# The `--` separates the logfile from the cargo args so both are preserved.
esp_cargo() {
    local log="${1:-check.log}"
    shift 1
    rm -f "$log"
    cargo "$@" > "$log" 2>&1
    echo "EXIT=$?" >> "$log"
    echo "logged to $log"
}
