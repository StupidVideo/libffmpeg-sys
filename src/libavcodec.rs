use libc::c_int;

#[link(name = "libavcodec_library")]
extern {
    struct AVFrame {}
    struct AVPacket {}
    struct AVCodecContext {}

    fn encode_frame(avctx: &AVCodecContext, pkt: &AVPacket, pict: &AVFrame, got_packet: c_int) -> c_int; // (AVCodecContext *avctx, AVPacket *pkt,const AVFrame *pict, int *got_packet)
}

fn encode_vid_to_tiff_img_seq(avctx: &AVCodecContext, pkt: &AVPacket, pict: &AVFrame, got_packet: c_int) {
    unsafe {
        encode_frame(*avctx, *pkt, *pict, *got_packet)
    }
}