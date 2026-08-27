# mrcal-sys

Raw FFI bindings to [mrcal](https://mrcal.secretsauce.net). The build script
downloads pinned tarballs of mrcal, libdogleg, and SuiteSparse (CHOLMOD +
AMD/CAMD/COLAMD/CCOLAMD/config), verifies their SHA256, and links them
statically. The only dynamic dependencies are OS-provided: Accelerate plus
libSystem/libc++ on macOS, `liblapack`/`libblas` on Linux.

Build requirements: a C/C++ toolchain, CMake, libclang, BLAS/LAPACK off
macOS, and network access on the first build —
`build-essential cmake libclang-dev liblapack-dev libblas-dev` on
Debian/Ubuntu, `gcc gcc-c++ cmake clang-devel lapack-devel blas-devel` on
Fedora.

For offline builds, point `MRCAL_SRC_DIR`, `DOGLEG_SRC_DIR`, and
`SUITESPARSE_SRC_DIR` at pre-extracted trees of the pinned versions.

BLAS/LAPACK comes from `pkg-config` (`lapack` + `blas`, then `openblas`),
falling back to `-llapack -lblas`. `MRCAL_LAPACK_LIBS` overrides it with a
space-separated list.

docs.rs has no network, so the build script ships `generated/bindings.rs` and
skips the native build when `DOCS_RS` is set. Refresh it when bumping
`MRCAL_VERSION`:

```sh
MRCAL_SYS_UPDATE_BINDINGS=1 cargo build -p mrcal-sys
```

Deliberately not compiled:

- `image.c` — image I/O, which would pull in stb/libpng/libjpeg. The
  `mrcal_image_*_t` types still exist; fill them from the `image` crate.
- `python-cameramodel-converter.c` — needs Python.h.
- `stereo-matching-libelas.cc` — optional libelas dependency.

`generated/` also holds what upstream generates at build time
(`minimath_generated.h` via perl, `cameramodel-parser_GENERATED.c` via re2c),
derived from mrcal sources (Apache-2.0, Copyright Caltech). Regenerate them
from the new tarball when bumping `MRCAL_VERSION`:

```sh
perl minimath/minimath_generate.pl > minimath_generated.h
re2c cameramodel-parser.re > cameramodel-parser_GENERATED.c
```

and re-check the `LIB_SOURCES` list in the upstream Makefile against
`MRCAL_C_SOURCES`/`MRCAL_CXX_SOURCES` in `build.rs`.

## Versioning

The crate version tracks `MRCAL_VERSION`, the C library it builds, but
Rust-only changes bump the patch past it. `MRCAL_VERSION` in `build.rs` is
authoritative.

## Licensing

The Rust code in this crate is Apache-2.0. The statically linked C components
are:

| Component | License |
|---|---|
| mrcal | Apache-2.0 |
| libdogleg | LGPL-3.0-or-later |
| CHOLMOD (Check, Cholesky, Utility, Partition) | LGPL-2.1-or-later |
| SuiteSparse_metis (embedded in CHOLMOD/Partition) | Apache-2.0 |
| AMD, CAMD, COLAMD, CCOLAMD, SuiteSparse_config | BSD-3-Clause |

The crate's `license` field is the composite of the above. CHOLMOD's GPL
modules (MatrixOps, Modify, Supernodal) are excluded via `CHOLMOD_GPL=OFF`, so
**no GPL code is linked**. That costs nothing: libdogleg hard-codes
`supernodal = 0`, and neither it nor libmrcal calls a GPL module.

The LGPL parts still matter when distributing binaries — LGPL §4(d) requires
letting recipients relink against modified versions, e.g. by shipping object
files or source.
