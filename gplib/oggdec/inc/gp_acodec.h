#ifndef __acodec_api_h__
#define __acodec_api_h__

/////////////////////////////////////////////////////////////////////////////
//		Type Definition
/////////////////////////////////////////////////////////////////////////////

/* 
 * audio decode parameter 
 */ 
typedef struct { 
    /* follow from parser */ 
    int			codec_id; /* see CODEC_ID_xxx in avcodec.h */ 
    int			sample_rate; 
    int			channels; 
    int			bit_rate; 
    int			block_align; /* for PCM */ 

	const char	*extradata; /* some audio head data*/
    int			extradata_size; /* head data size */

    const char	*es_data; /* element stream data */
    int			es_size;  /* element stream data size, unit in byte*/ 

    const char	*Ring;
    int			RingSize;
    int			RingRI;
    int			RingWI;

    /* follow from decode control part */ 
    char		*audec_buf; /* audio decoder instance buffer, align to 4 bytes boundary */ 

    /* follow from decoder */ 
    int		out_fmt; 
    int		out_sample_rate; 
    int		out_channels; 
    int		out_bit_rate; 
//<<add 160104
	unsigned int FileCnt;
	unsigned int FileLen;
//add 160104>>
} audec_param_t;


/* 
 * audio decode interface 
 */ 
typedef struct audec_interface_s 
{ 
    /* @description: get audio decoder instrance size 
        * @return: audio decoder instrance size. 
       */ 
    unsigned int (*instance_size)(void); 
    /* @description: initialize audio decoder 
        * @param: *adp[in] the pointer of audio decode parameter struct 
        * @return: On error a negative value is returned, otherwise 0. 
        */ 
    int (*init)(audec_param_t *adp); 
    /* @description: decode one audio frame 
        * @param: *adp[in] the pointer of audio es infomation 
        * @param: out_buf[out] the pointer of the output buffer 
        * @param: out_size[in\out] input the output buffer size, return the output pcm size 
        * @return: On error a negative value is returned, otherwise the number of bytes 
        * used or zero if no frame could be decompressed. 
        */ 
    int (*dec)(audec_param_t *adp, void *out_buf, int *out_size); 
    /* @description: uninitialize audio decoder 
        * @param: *adp[in] the pointer of audio decode parameter struct 
        * @return: On error a negative value is returned, otherwise 0. 
        */ 
    int (*uninit)(audec_param_t *adp);
} audec_interface_t;

#endif //__ap_def_h__