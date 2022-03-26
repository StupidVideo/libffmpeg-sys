use libc::{c_int, c_uint, c_void}; // c_void: Equivalent to C's void type when used as a pointer.
use std::mem::MaybeUninit;

#[link(name = "libavcodec_library")]
extern {
    // struct AVFrame;
    // struct AVPacket;

    const FF_COMPRESSION_DEFAULT: u8 = -1;

    struct AVCodecContext {
        /**
         * information on struct for av_log
         * - set by avcodec_alloc_context3
         */
        av_class: *mut AVClass,                                     // const AVClass *av_class;
        log_level_offset: c_int,                                    // int log_level_offset;

        codec_type: AVMediaType,    /* see AVMEDIA_TYPE_xxx */      // enum AVMediaType codec_type;
        codec: *mut AVCodec,                                        // const struct AVCodec  *codec;
        codec_id: AVCodecID,        /* see AV_CODEC_ID_xxx */       // enum AVCodecID     codec_id;

        /**
         * fourcc (LSB first, so "ABCD" -> ('D'<<24) + ('C'<<16) + ('B'<<8) + 'A').
         * This is used to work around some encoder bugs.
         * A demuxer should set this to what is stored in the field used to identify the codec.
         * If there are multiple such fields in a container then the demuxer should choose the one
         * which maximizes the information about the used codec.
         * If the codec tag field in a container is larger than 32 bits then the demuxer should
         * remap the longer ID to 32 bits with a table or other structure. Alternatively a new
         * extra_codec_tag + size could be added but for this a clear advantage must be demonstrated
         * first.
         * - encoding: Set by user, if not then the default based on codec_id will be used.
         * - decoding: Set by user, will be converted to uppercase by libavcodec during init.
         */
        codec_tag: c_uint,                                          // unsigned int codec_tag;
        priv_data: *mut c_void,                                     // void *priv_data;

        /**
         * Private context used for internal data.
         *
         * Unlike priv_data, this is not codec-specific. It is used in general
         * libavcodec functions.
         */
        internal: *mut AVCodecInternal,                             // struct AVCodecInternal *internal;

        /**
         * Private data of the user, can be used to carry app specific stuff.
         * - encoding: Set by user.
         * - decoding: Set by user.
         */
        opaque: *mut c_void,                                        // void *opaque;

        /**
         * the average bitrate
         * - encoding: Set by user; unused for constant quantizer encoding.
         * - decoding: Set by user, may be overwritten by libavcodec
         *             if this info is available in the stream
         */
        bitrate: i64,                                               // int64_t bit_rate;

        /**
         * number of bits the bitstream is allowed to diverge from the reference.
         *           the reference can be CBR (for CBR pass1) or VBR (for pass2)
         * - encoding: Set by user; unused for constant quantizer encoding.
         * - decoding: unused
         */
        bit_rate_tolerance: c_int,                                  // int bit_rate_tolerance;

        /**
         * Global quality for codecs which cannot change it per frame.
         * This should be proportional to MPEG-1/2/4 qscale.
         * - encoding: Set by user.
         * - decoding: unused
         */
        global_quality: c_int,                                      // int global_quality;

        /**
         * - encoding: Set by user.
         * - decoding: unused
         */
        compression_level: c_int,                                   // int compression_level;
        // const FF_COMPRESSION_DEFAULT defined above               // #define FF_COMPRESSION_DEFAULT -1
        /**
         * AV_CODEC_FLAG_*.
         * - encoding: Set by user.
         * - decoding: Set by user.
         */
        flags: c_int,                                               // int flags;
    }

    fn encode_frame(avctx: &AVCodecContext, pkt: &AVPacket, pict: &AVFrame, got_packet: c_int) -> c_int; // (AVCodecContext *avctx, AVPacket *pkt,const AVFrame *pict, int *got_packet)
}

fn encode_vid_to_tiff_img_seq(avctx: &AVCodecContext, pkt: &AVPacket, pict: &AVFrame, got_packet: c_int) {
    
    unsafe {
        encode_frame(*avctx, *pkt, *pict, *got_packet)
    }
}