// The lints bindgen output needs vary by compiler version
#![allow(unknown_lints)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(unsafe_op_in_unsafe_fn)]
#![allow(clippy::all)]
#![allow(unnecessary_transmutes)]
// Some lens-model config structs are legitimately empty in C
#![allow(improper_ctypes)]
#![allow(rustdoc::broken_intra_doc_links)]
#![allow(rustdoc::invalid_html_tags)]
#![allow(rustdoc::bare_urls)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
