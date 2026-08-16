// RADAR-TX build script (standard esp-idf-template pattern).
//
// esp-idf-sys 0.37 exposes its native link args (the ldproxy hints and the
// `@linker_args.txt` response file) as `DEP_ESP_IDF_*_EMBUILD_LINK_ARGS`
// metadata. Cargo does NOT propagate `cargo:rustc-link-arg` from a dependency
// rlib to the final binary, so without this build script the final link of
// `radar_tx` never receives `--ldproxy-linker <gcc>`, `--ldproxy-cwd <dir>` or
// `@<out>/linker_args.txt` and ldproxy panics ("Cannot locate argument
// '--ldproxy-linker <linker>'"). This one-liner re-emits those args for the bin
// target. See .cargo/config.toml `[target.xtensa-esp32-espidf] linker`.
fn main() {
    embuild::espidf::sysenv::output();
}
