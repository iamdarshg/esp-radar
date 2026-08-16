// Standard esp-idf build script: re-emits esp-idf-sys's native link args
// (--ldproxy-linker / --ldproxy-cwd / @linker_args.txt) from this bin
// crate's own build script. Cargo does NOT propagate cargo:rustc-link-arg
// from a dependency rlib to the final binary link; this is the canonical
// mechanism (see firmware/radar_tx/build.rs).
fn main() {
    embuild::espidf::sysenv::output();
}
