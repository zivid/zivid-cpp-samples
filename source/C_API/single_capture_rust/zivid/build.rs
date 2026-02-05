fn main() {
    println!("cargo:rustc-link-search=../../../../build/C_API/CoreCAPI/");
    println!("cargo:rustc-link-lib=dylib=CoreCAPI");
}
