#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use libc::*;

include!("./bindings/libavcodec/bindings.rs", 
    "./bindings/libavutil/bindings.rs");
