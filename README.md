# VideoMain

To generate bindings for libavutil, run: 
`bindgen include/avutil.h -o src/libavutil/bindings.rs`
This will generate much, much more than what's necessary. However, it will do for now (will clean up in later versions).

## TODO
[] For functions that have return values for errors, let's do ourselves a favor now and ensure the 
#[must_use] annotation is attached to each one. This will at least give some indication if callers forget to check the return value for errors, and it will help later when everything gets wrapped in safe layers.