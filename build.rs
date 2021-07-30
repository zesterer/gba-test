use std::{
    path::Path,
    fs::read_dir,
};

fn check_data<P: AsRef<Path>>(path: P) {
    for entry in read_dir(path).unwrap().map(|e| e.unwrap()) {
        if entry.metadata().unwrap().is_dir() {
            check_data(&entry.path());
        } else {
            println!("cargo:rerun-if-changed={:?}", entry.path());
        }
    }
}

fn main() {
    println!("cargo:rerun-if-changed=crt0.s");
    println!("cargo:rerun-if-changed=linker.ld");

    check_data("data/");
}
