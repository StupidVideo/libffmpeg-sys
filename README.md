# VideoMain

## FFMPEG
See `include/ffmpeg.txt` for diagram.

## Invoking Bindgen
To generate bindings for libavutil, run: 
`bindgen include/libavcodec/avcodec.h -o src/libavcodec/bindings.rs`
`bindgen include/libavutil/avutil.h -o src/libavutil/bindings.rs`
This will generate much, much more than what's necessary. However, it will do for now (will clean up in later versions).

## TODO
[] For functions that have return values for errors, let's do ourselves a favor now and ensure the 
#[must_use] annotation is attached to each one. This will at least give some indication if callers forget to check the return value for errors, and it will help later when everything gets wrapped in safe layers.

[] preferred_stereo_downmix() - #include <ac3enc.h>