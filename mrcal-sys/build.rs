use sha2::{Digest, Sha256};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

const MRCAL_VERSION: &str = "2.5.2";
const MRCAL_SHA256: &str = "f0a8471fc5dc3bba3719c2f8aaf968d6fa074903575ef35ddbae33ad5ea1ccc1";

const DOGLEG_VERSION: &str = "0.18";
const DOGLEG_SHA256: &str = "d97ef0c149463f84e9bd40c8852da444605a38bac432b5b2774de3dd15180bab";

const SUITESPARSE_VERSION: &str = "7.12.2";
const SUITESPARSE_SHA256: &str = "679412daa5f69af96d6976595c1ac64f252287a56e98cc4a8155d09cc7fd69e8";

// LIB_SOURCES from the upstream Makefile, minus image.c (drops stb/libpng/
// libjpeg), python-cameramodel-converter.c (needs Python.h) and
// stereo-matching-libelas.cc (optional libelas).
const MRCAL_C_SOURCES: &[&str] = &[
    "mrcal.c",
    "opencv.c",
    "uncertainty.c",
    "stereo.c",
    "poseutils.c",
    "poseutils-opencv.c",
    "traverse-sensor-links.c",
    "cameramodel-parser_GENERATED.c",
];
const MRCAL_CXX_SOURCES: &[&str] = &[
    "poseutils-uses-autodiff.cc",
    "triangulation.cc",
    "cahvore.cc",
    "heap.cc",
];

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // docs.rs has no network and needs no linkable library
    if env::var_os("DOCS_RS").is_some() {
        fs::copy(
            manifest.join("generated/bindings.rs"),
            out.join("bindings.rs"),
        )
        .expect("checked-in bindings.rs for docs.rs builds");
        return;
    }

    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=generated/minimath_generated.h");
    println!("cargo:rerun-if-changed=generated/cameramodel-parser_GENERATED.c");
    println!("cargo:rerun-if-env-changed=MRCAL_SRC_DIR");
    println!("cargo:rerun-if-env-changed=DOGLEG_SRC_DIR");
    println!("cargo:rerun-if-env-changed=SUITESPARSE_SRC_DIR");
    println!("cargo:rerun-if-env-changed=MRCAL_LAPACK_LIBS");

    let mrcal_src = source_tree(
        "MRCAL_SRC_DIR",
        &format!("https://github.com/dkogan/mrcal/archive/refs/tags/v{MRCAL_VERSION}.tar.gz"),
        MRCAL_SHA256,
        &format!("mrcal-{MRCAL_VERSION}"),
        &out,
    );
    let dogleg_src = source_tree(
        "DOGLEG_SRC_DIR",
        &format!("https://github.com/dkogan/libdogleg/archive/refs/tags/v{DOGLEG_VERSION}.tar.gz"),
        DOGLEG_SHA256,
        &format!("libdogleg-{DOGLEG_VERSION}"),
        &out,
    );
    let suitesparse_src = source_tree(
        "SUITESPARSE_SRC_DIR",
        &format!(
            "https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/refs/tags/v{SUITESPARSE_VERSION}.tar.gz"
        ),
        SUITESPARSE_SHA256,
        &format!("SuiteSparse-{SUITESPARSE_VERSION}"),
        &out,
    );

    let ss_install = build_suitesparse(&suitesparse_src, &out);
    let ss_include = ss_install.join("include");

    // The OUT_DIR tree is ours to mutate; drop in what upstream generates
    // with perl/re2c
    fs::copy(
        manifest.join("generated/minimath_generated.h"),
        mrcal_src.join("minimath/minimath_generated.h"),
    )
    .unwrap();
    fs::copy(
        manifest.join("generated/cameramodel-parser_GENERATED.c"),
        mrcal_src.join("cameramodel-parser_GENERATED.c"),
    )
    .unwrap();

    cc::Build::new()
        .files(MRCAL_C_SOURCES.iter().map(|f| mrcal_src.join(f)))
        .include(&mrcal_src)
        .include(&dogleg_src)
        .include(&ss_include)
        .std("gnu99")
        .opt_level(3)
        .warnings(false)
        .compile("mrcal_c");

    cc::Build::new()
        .cpp(true)
        .files(MRCAL_CXX_SOURCES.iter().map(|f| mrcal_src.join(f)))
        .include(&mrcal_src)
        .include(&dogleg_src)
        .include(&ss_include)
        .opt_level(3)
        .warnings(false)
        .compile("mrcal_cxx");

    cc::Build::new()
        .file(dogleg_src.join("dogleg.c"))
        .include(&ss_include)
        .opt_level(3)
        .warnings(false)
        .compile("dogleg");

    for dir in ["lib", "lib64"] {
        let path = ss_install.join(dir);
        if path.is_dir() {
            println!("cargo:rustc-link-search=native={}", path.display());
        }
    }
    for lib in [
        "cholmod",
        "camd",
        "ccolamd",
        "amd",
        "colamd",
        "suitesparseconfig",
    ] {
        println!("cargo:rustc-link-lib=static={lib}");
    }

    link_lapack();

    let bindings = bindgen::Builder::default()
        .header(manifest.join("wrapper.h").to_str().unwrap().to_owned())
        .clang_arg(format!("-I{}", mrcal_src.display()))
        .allowlist_function("mrcal_.*")
        .allowlist_type("(mrcal|MRCAL)_.*")
        .allowlist_var("(mrcal|MRCAL)_.*")
        // Declared in image.h but image.c is not compiled
        .blocklist_function("mrcal_image_.*_(load|save)")
        .derive_default(true)
        .layout_tests(false)
        .generate()
        .expect("bindgen failed on mrcal headers");
    bindings
        .write_to_file(out.join("bindings.rs"))
        .expect("failed to write bindings.rs");

    // Refresh the copy docs.rs uses
    if env::var_os("MRCAL_SYS_UPDATE_BINDINGS").is_some() {
        fs::copy(
            out.join("bindings.rs"),
            manifest.join("generated/bindings.rs"),
        )
        .expect("failed to refresh generated/bindings.rs");
    }
}

/// Link the BLAS/LAPACK that CHOLMOD and mrcal's direct dgesdd_/dgeev_/
/// dpptrf_/dpptrs_ calls need. `MRCAL_LAPACK_LIBS` overrides the choice with
/// a space-separated library list.
fn link_lapack() {
    if let Ok(libs) = env::var("MRCAL_LAPACK_LIBS") {
        for lib in libs.split_whitespace() {
            println!("cargo:rustc-link-lib={lib}");
        }
        return;
    }

    // System frameworks cannot be statically linked; everything else is static.
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        println!("cargo:rustc-link-lib=framework=Accelerate");
        return;
    }

    // Reference LAPACK ships lapack.pc/blas.pc; OpenBLAS ships one library
    if pkg_config_libs(&["lapack", "blas"]) || pkg_config_libs(&["openblas"]) {
        return;
    }
    println!("cargo:rustc-link-lib=lapack");
    println!("cargo:rustc-link-lib=blas");
}

/// Emit link flags for a set of pkg-config modules, or nothing if any is
/// unknown.
fn pkg_config_libs(modules: &[&str]) -> bool {
    let mut flags = Vec::new();
    for module in modules {
        let Ok(output) = std::process::Command::new("pkg-config")
            .args(["--libs", module])
            .output()
        else {
            return false;
        };
        if !output.status.success() {
            return false;
        }
        flags.push(String::from_utf8_lossy(&output.stdout).into_owned());
    }
    for flag in flags.iter().flat_map(|f| f.split_whitespace()) {
        if let Some(lib) = flag.strip_prefix("-l") {
            println!("cargo:rustc-link-lib={lib}");
        } else if let Some(dir) = flag.strip_prefix("-L") {
            println!("cargo:rustc-link-search=native={dir}");
        }
    }
    true
}

/// Materialize a source tree under OUT_DIR/src/<root_dir>, either copied from
/// `env_override` (offline builds) or downloaded as a checksum-verified
/// tarball.
fn source_tree(env_override: &str, url: &str, sha256: &str, root_dir: &str, out: &Path) -> PathBuf {
    let src_root = out.join("src");
    let dest = src_root.join(root_dir);
    let marker = dest.join(".mrcal-sys-ok");
    if marker.exists() {
        return dest;
    }
    if dest.exists() {
        fs::remove_dir_all(&dest).unwrap();
    }
    fs::create_dir_all(&src_root).unwrap();

    if let Ok(local) = env::var(env_override) {
        copy_tree(Path::new(&local), &dest);
    } else {
        let bytes = download(url);
        let digest = Sha256::digest(&bytes);
        let digest_hex: String = digest.iter().map(|b| format!("{b:02x}")).collect();
        assert_eq!(
            digest_hex, sha256,
            "SHA256 mismatch for {url}: got {digest_hex}, expected {sha256}"
        );
        let gz = flate2::read::GzDecoder::new(bytes.as_slice());
        tar::Archive::new(gz)
            .unpack(&src_root)
            .unwrap_or_else(|e| panic!("failed to extract {url}: {e}"));
        assert!(
            dest.is_dir(),
            "tarball {url} did not contain expected root directory {root_dir}"
        );
    }

    fs::write(&marker, "").unwrap();
    dest
}

fn download(url: &str) -> Vec<u8> {
    let mut response = ureq::get(url)
        .call()
        .unwrap_or_else(|e| panic!("failed to download {url}: {e}"));
    response
        .body_mut()
        .with_config()
        .limit(512 * 1024 * 1024)
        .read_to_vec()
        .unwrap_or_else(|e| panic!("failed to read body of {url}: {e}"))
}

fn copy_tree(from: &Path, to: &Path) {
    fs::create_dir_all(to).unwrap();
    for entry in fs::read_dir(from).unwrap() {
        let entry = entry.unwrap();
        let target = to.join(entry.file_name());
        // Resolve through symlinks; skip dangling ones rather than panicking
        let Ok(metadata) = fs::metadata(entry.path()) else {
            continue;
        };
        if metadata.is_dir() {
            copy_tree(&entry.path(), &target);
        } else {
            fs::copy(entry.path(), &target).unwrap();
        }
    }
}

fn build_suitesparse(src: &Path, out: &Path) -> PathBuf {
    // cmake::Config installs into OUT_DIR; skip the slow cmake invocation on
    // reruns where the artifacts already exist. The libdir varies by distro.
    let installed = ["lib", "lib64"]
        .iter()
        .any(|dir| out.join(dir).join("libcholmod.a").exists());
    if installed {
        return out.to_path_buf();
    }
    cmake::Config::new(src)
        .profile("Release")
        .define(
            "SUITESPARSE_ENABLE_PROJECTS",
            "suitesparse_config;amd;camd;ccolamd;colamd;cholmod",
        )
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_STATIC_LIBS", "ON")
        .define("SUITESPARSE_USE_FORTRAN", "OFF")
        // Excludes CHOLMOD's GPL modules, keeping the artifact GPL-free.
        // Nothing here can use them anyway: libdogleg forces supernodal=0 and
        // calls only LGPL Cholesky/Utility routines.
        .define("CHOLMOD_GPL", "OFF")
        .define("SUITESPARSE_USE_CUDA", "OFF")
        .define("SUITESPARSE_USE_OPENMP", "OFF")
        .define("SUITESPARSE_DEMOS", "OFF")
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "ON")
        .build()
}
