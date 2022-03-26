use libc::{c_int, c_uint, c_void}; // c_void: Equivalent to C's void type when used as a pointer.
use std::mem::MaybeUninit;

#[link(name = "libavcodec_library")]
extern {
    // struct AVFrame;
    // struct AVPacket;

    const FF_COMPRESSION_DEFAULT: u8 = -1;                          // #define FF_COMPRESSION_DEFAULT -1

    // struct AVRational; // used in AVCodeContext
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
    
        /**
         * AV_CODEC_FLAG2_*
         * - encoding: Set by user.
         * - decoding: Set by user.
         */
        flags2: c_int,                                              // int flags2;

        /**
         * some codecs need / can use extradata like Huffman tables.
         * MJPEG: Huffman tables
         * rv10: additional flags
         * MPEG-4: global headers (they can be in the bitstream or here)
         * The allocated memory should be AV_INPUT_BUFFER_PADDING_SIZE bytes larger
         * than extradata_size to avoid problems if it is read with the bitstream reader.
         * The bytewise contents of extradata must not depend on the architecture or CPU endianness.
         * Must be allocated with the av_malloc() family of functions.
         * - encoding: Set/allocated/freed by libavcodec.
         * - decoding: Set/allocated/freed by user.
         */
        extradata: *mut u8,                                         // uint8_t *extradata;
        extradate_size: c_int,                                      // int extradata_size;

        /**
         * This is the fundamental unit of time (in seconds) in terms
         * of which frame timestamps are represented. For fixed-fps content,
         * timebase should be 1/framerate and timestamp increments should be
         * identically 1.
         * This often, but not always is the inverse of the frame rate or field rate
         * for video. 1/time_base is not the average frame rate if the frame rate is not
         * constant.
         *
         * Like containers, elementary streams also can store timestamps, 1/time_base
         * is the unit in which these timestamps are specified.
         * As example of such codec time base see ISO/IEC 14496-2:2001(E)
         * vop_time_increment_resolution and fixed_vop_rate
         * (fixed_vop_rate == 0 implies that it is different from the framerate)
         *
         * - encoding: MUST be set by user.
         * - decoding: the use of this field for decoding is deprecated.
         *             Use framerate instead.
         */
        time_base: AVRational,                                  // AVRational time_base;

        /**
         * For some codecs, the time base is closer to the field rate than the frame rate.
         * Most notably, H.264 and MPEG-2 specify time_base as half of frame duration
         * if no telecine is used ...
         *
         * Set to time_base ticks per frame. Default 1, e.g., H.264/MPEG-2 set it to 2.
         */
        ticks_per_frame: c_int,                                 // int ticks_per_frame;

        /**
         * Codec delay.
         *
         * Encoding: Number of frames delay there will be from the encoder input to
         *           the decoder output. (we assume the decoder matches the spec)
         * Decoding: Number of frames delay in addition to what a standard decoder
         *           as specified in the spec would produce.
         *
         * Video:
         *   Number of frames the decoded output will be delayed relative to the
         *   encoded input.
         *
         * Audio:
         *   For encoding, this field is unused (see initial_padding).
         *
         *   For decoding, this is the number of samples the decoder needs to
         *   output before the decoder's output is valid. When seeking, you should
         *   start decoding this many samples prior to your desired seek point.
         *
         * - encoding: Set by libavcodec.
         * - decoding: Set by libavcodec.
         */
        delay: c_int,                                           // int delay;


        /* video only */
        /**
         * picture width / height.
         *
         * @note Those fields may not match the values of the last
         * AVFrame output by avcodec_receive_frame() due frame
         * reordering.
         *
         * - encoding: MUST be set by user.
         * - decoding: May be set by the user before opening the decoder if known e.g.
         *             from the container. Some decoders will require the dimensions
         *             to be set by the caller. During decoding, the decoder may
         *             overwrite those values as required while parsing the data.
         */
        width: c_int,                                           // int width, height;
        height: c_int,                                          //
    }

    fn encode_frame(avctx: &AVCodecContext, pkt: &AVPacket, pict: &AVFrame, got_packet: c_int) -> c_int; // (AVCodecContext *avctx, AVPacket *pkt,const AVFrame *pict, int *got_packet)
}

fn encode_vid_to_tiff_img_seq(avctx: &AVCodecContext, pkt: &AVPacket, pict: &AVFrame, got_packet: c_int) {
    
    unsafe {
        encode_frame(*avctx, *pkt, *pict, *got_packet)
    }
}