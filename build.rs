// build.rs

fn main() {
    cc::Build::new()
        .file("tiffenc.c")
        .file("avcodec.c")
        .compile("avcodec_library")
}