#ifndef __GPLIB_H__
#define __GPLIB_H__

#include "project.h"

// sensor frame end action
#define SENSOR_FEA_VIDEO_ENCODE			0
#define SENSOR_FEA_OID					1

#if 0
//Time and data reminder
typedef struct
{
    INT32S tm_sec;  /* 0-59 */
    INT32S tm_min;  /* 0-59 */
    INT32S tm_hour; /* 0-23 */
    INT32S tm_mday; /* 1-31 */
    INT32S tm_mon;  /* 1-12 */
    INT32S tm_year;
    INT32S tm_wday; /* 0-6 Sunday,Monday,Tuesday,Wednesday,Thursday,Friday,Saturday */
}TIME_T;
#endif

/* MP3 */
/////////////////////////////////////////////////////////////////////////////
//		Constant Definition
/////////////////////////////////////////////////////////////////////////////
#define MP3_DEC_FRAMESIZE					1152	// ???
#define MP3_DEC_BITSTREAM_BUFFER_SIZE   	4096    // size in bytes
#define MP3_DEC_MEMORY_SIZE 			13500
#define MP3_DECODE_RAM     				7176
#define MP3_WORKING_MEM                 0x20000000

/////////////////////////////////////////////////////////////////////////////
//		Error Code
/////////////////////////////////////////////////////////////////////////////
#define MP3_DEC_ERR_NONE			0x00000000	/* no error */

#define MP3_DEC_ERR_BUFLEN	   	   	0x80000001	/* input buffer too small (or EOF) */
#define MP3_DEC_ERR_BUFPTR	   	   	0x80000002	/* invalid (null) buffer pointer */

#define MP3_DEC_ERR_NOMEM	   	   	0x80000031	/* not enough memory */

#define MP3_DEC_ERR_LOSTSYNC	   	0x80000101	/* lost synchronization */
#define MP3_DEC_ERR_BADLAYER	   	0x80000102	/* reserved header layer value */
#define MP3_DEC_ERR_BADBITRATE	   	0x80000103	/* forbidden bitrate value */
#define MP3_DEC_ERR_BADSAMPLERATE  	0x80000104	/* reserved sample frequency value */
#define MP3_DEC_ERR_BADEMPHASIS	   	0x80000105	/* reserved emphasis value */
#define MP3_DEC_ERR_BADMPEGID		0x80000106	//for error mpegid add by zgq on 20080508

#define MP3_DEC_ERR_BADCRC	   	   	0x80000201	/* CRC check failed */
#define MP3_DEC_ERR_BADBITALLOC	   	0x80000211	/* forbidden bit allocation value */
#define MP3_DEC_ERR_BADSCALEFACTOR  0x80000221	/* bad scalefactor index */
#define MP3_DEC_ERR_BADMODE         0x80000222	/* bad bitrate/mode combination */
#define MP3_DEC_ERR_BADFRAMELEN	    0x80000231	/* bad frame length */
#define MP3_DEC_ERR_BADBIGVALUES    0x80000232	/* bad big_values count */
#define MP3_DEC_ERR_BADBLOCKTYPE    0x80000233	/* reserved block_type */
#define MP3_DEC_ERR_BADSCFSI	    0x80000234	/* bad scalefactor selection info */
#define MP3_DEC_ERR_BADDATAPTR	    0x80000235	/* bad main_data_begin pointer */
#define MP3_DEC_ERR_BADPART3LEN	    0x80000236	/* bad audio data length */
#define MP3_DEC_ERR_BADHUFFTABLE    0x80000237	/* bad Huffman table select */
#define MP3_DEC_ERR_BADHUFFDATA	    0x80000238	/* Huffman data overrun */
#define MP3_DEC_ERR_BADSTEREO	    0x80000239	/* incompatible block_type for JS */

/////////////////////////////////////////////////////////////////////////////
//		Function Definition
/////////////////////////////////////////////////////////////////////////////

// MP3 Decoder Version
// @return  return version of mp3 decoder library
extern const unsigned char * mp3_dec_get_version(void);

// MP3 Decoder Initial
// @param  *p_workmem: pointer to working memory
// @param  *p_bsbuf: pointer to bitstream buffer
// @param  *ram: pointer to ram buffer
// @return  return 0 (success), others(fail)
extern int mp3_dec_init(char *p_workmem, unsigned char *p_bsbuf, char *ram);

// tell the size of bitstream buffer to decoder
// @param   *p_workmem: pointer to working memory
// @param   wi: write index of bitstream buffer
// @return  return 0 (success), others(fail)
extern int mp3_dec_parsing(char *p_workmem, unsigned int wi);

// MP3 Decoder
// @param   *p_workmem: pointer to working memory
// @param   *p_pcmbuf: pointer to PCM buffer
// @param   wi: write index of bitstream buffer
// @return  return the number of sample. if return 0, decoder need more bitstream
extern int mp3_dec_run(char *p_workmem, short *p_pcmbuf, unsigned int wi);

// Get Read Index
// @param  *p_workmem: pointer to working memory
// @return  return read index of bitstream buffer
extern int mp3_dec_get_ri(char *p_workmem);

// Set Read Index
// @param  *p_workmem: pointer to working memory
// @param  ri: read index of bitstream buffer
extern void mp3_dec_set_ri(char *mp3dec_workmem, int ri);

// Set bitstream size
// @param  *p_workmem: pointer to working memory
// @param  bs_buf_size: size of bitstream buffer
extern void mp3_dec_set_bs_buf_size(char *mp3dec_workmem, int bs_buf_size);

// Get mpeg id
// @param   *p_workmem: pointer to working memory
// @return  return MPEG id: 1 or 2 or 2.5
extern const char *mp3_dec_get_mpegid(char *p_workmem);

// Get the size of MP3 Decoder Working memory
// @return   return size of MP3 Decoder Working memory
extern int mp3_dec_get_mem_block_size (void);

// get error number.
// @param   *p_workmem: pointer to working memory
// @return  return error number
extern int mp3_dec_get_errno(char *p_workmem);

// get layer.
// @param   *p_workmem: pointer to working memory
// @return  return number of layer
extern int mp3_dec_get_layer(char *p_workmem);

// get channel.
// @param   *p_workmem: pointer to working memory
// @return  return 1(mono), 2(stereo)
extern int mp3_dec_get_channel(char *p_workmem);

// get bitrate in kbps.
// @param   *p_workmem: pointer to working memory
// @return  return bitrate in kbps
extern int mp3_dec_get_bitrate(char *p_workmem);

// get sampling rate in Hz.
// @param   *p_workmem: pointer to working memory
// @return  return sampling rate in Hz
extern int mp3_dec_get_samplerate(char *p_workmem);

// set EQ table
// @param   *p_workmem: pointer to working memory
// @param   *eqtable: pointer to EQ Table (short EQ_Tab[12])
extern void mp3_dec_set_EQ(unsigned char *mp3dec_workmem, unsigned short *EQ_table);

// set volume gain, Q.14 (1 = 0x4000)
extern void mp3_dec_set_volume(char *mp3dec_workmem, unsigned short vol);


//---------------------------------------------------------------------------
#ifndef __MP3ENC_H__
#define __MP3ENC_H__

#define  MP3_ENC_WORKMEM_SIZE  31704
// error code
#define MP3ENC_ERR_INVALID_CHANNEL		0x80000001
#define MP3ENC_ERR_INVALID_SAMPLERATE	0x80000002
#define MP3ENC_ERR_INVALID_BITRATE		0x80000003

extern int mp3enc_init(
	void *pWorkMem,
	int nChannels,
	int nSamplesPreSec,
	int nKBitsPreSec,
	int Copyright,
	char *Ring,
	int RingBufSize,
	int RingWI);

extern int mp3enc_encframe(void *pWorkMem, const short *PCM);
extern int mp3enc_end(void *pWorkMem);
const char *mp3enc_GetErrString(int ErrCode);
const char *mp3enc_GetVersion(void);
int mp3enc_GetWorkMemSize(void);

#endif	// __MP3ENC_H__

//---------------------------------------------------------------------------
/* WMA */
#define WMA_DEC_FRAMESIZE        2048
#define WMA_DEC_MEMORY_SIZE      8192*2
#define WMA_DEC_BITSTREAM_BUFFER_SIZE 16384

/*
* Function Name :  wma_dec_run
* Syntax : int wma_dec_run(char *p_workmem, short *p_pcmbuf, int wi);
* Purpose :  decode a wma frame
* Parameters : char *p_workmem: allocated working memory
*							 short *p_pcmbuf: input pcm buffer address
*              int wi: write index of bitstream buffer
* Return : the number of sample. if return 0, decoder need more bitstream
*/
extern int			wma_dec_run(char *p_workmem, short *p_pcmbuf, int wi);	//decode function

/*
* Function Name :  wma_dec_init
* Syntax : int wma_dec_init(char* wmadec_workmem, unsigned char* bs_buf, char *wmahw_mem, int bs_len);
* Purpose :  Initializing the memory used by wma decoder
* Parameters : char *wmadec_workmem: allocated working memory pointer
*							 unsigned char *bs_buf: pointer to the header of in buffer
*              char *wmahw_mem: allocated working memory
*							 int bs_len: ring buffer length
* Return : 0
*/
extern int  		wma_dec_init(char* wmadec_workmem, unsigned char* bs_buf, char *wmahw_mem, int bs_len);//initialize

/*
* Function Name :  wma_dec_parsing
* Syntax : int wma_dec_parsing(char *wmadec_workmem,int wi);
* Purpose :  finding sync word and parsing wma header information
* Parameters : char *wmadec_workmem: allocated working memory pointer
*							 int wi: write index of bitstream buffer
* Return : 0 is ok; other vaule is failed. Check error code.
*/
extern int			wma_dec_parsing(char *wmadec_workmem,int wi);			//parsing header of the wma file

/*
* Function Name :  wma_dec_get_ri
* Syntax : int wma_dec_get_ri(char *wmadec_workmem);
* Purpose :  get read index of bitstream ring buffer
* Parameters : char *wmadec_workmem: allocated working memory pointer
* Return : read index of bit stream ring buffer
*/
extern int			wma_dec_get_ri(char *wmadec_workmem);					//get read index of the ringbuf

/*
* Function Name :  wma_dec_get_errno
* Syntax : int wma_dec_get_errno(char *wmadec_workmem);
* Purpose :  get error code number
* Parameters : char *wmadec_workmem: allocated working memory pointer
* Return : Error code number
*/
extern int			wma_dec_get_errno(char *wmadec_workmem);				//get error information

/*
* Function Name :  wma_dec_get_version
* Syntax : const unsigned char * wma_dec_get_version(void);
* Purpose :  get version
* Parameters : none
* Return : version of wma decoder library
*/
extern const unsigned char * wma_dec_get_version(void);						//get version of the project

/*
* Function Name :  wma_dec_get_mem_block_size
* Syntax : int wma_dec_get_mem_block_size(void);
* Purpose :  get memory size of wma decoder working memory
* Parameters :none
* Return : get wma decoder working memory size
*/
extern int			wma_dec_get_mem_block_size(void);						//get memory block size

/*
* Function Name :  wma_dec_get_samplerate
* Syntax : int wma_dec_get_samplerate(char *wmadec_workmem);
* Purpose :  get sample rate
* Parameters : char *wmadec_workmem: allocated working memory pointer
* Return : sample rate in Hz
*/
extern int			wma_dec_get_samplerate(char *wmadec_workmem);			//get samplerate of the wma file

/*
* Function Name :  wma_dec_get_channel
* Syntax : int wma_dec_get_channel(char *wmadec_workmem);
* Purpose :  get channel
* Parameters : char *wmadec_workmem: allocated working memory pointer
* Return : 1: mono; 2: stereo
*/
extern int			wma_dec_get_channel(char *wmadec_workmem);				//get channel of the wma file

/*
* Function Name :  wma_dec_get_bitspersample
* Syntax : int wma_dec_get_bitspersample(char *wmadec_workmem);
* Purpose :  get bits per sample
* Parameters : char *wmadec_workmem: allocated working memory pointer
* Return : bits per sample
*/
extern int			wma_dec_get_bitspersample(char *wmadec_workmem);		//get the bits per sample

/*
* Function Name :  wma_dec_get_bitrate
* Syntax : int wma_dec_get_bitrate(char *wmadec_workmem);
* Purpose :  get bit rate
* Parameters : char *wmadec_workmem: allocated working memory pointer
* Return : bit rate in bps
*/
extern int			wma_dec_get_bitrate(char *wmadec_workmem);				//get the bitrate of the wma file

/*
* Function Name :  wma_dec_set_EQ
* Syntax : void wma_dec_set_EQ(char *wmadec_workmem, unsigned short *p_EQ_table);
* Purpose :  set EQ parameters of wma
* Parameters : char *wmadec_workmem: allocated working memory pointer
*							 unsigned short *p_EQ_table: pointer to EQ Table
* Return : none
*/
extern void			wma_dec_set_EQ(char *wmadec_workmem, unsigned short *p_EQ_table);//set EQ parameters of the wma file

/*
* Function Name :  wma_dec_get_playtime
* Syntax : int wma_dec_get_playtime(char *wmadec_workmem);
* Purpose :  get the play time of the wma file
* Parameters : char *wmadec_workmem: allocated working memory pointer
* Return : get the play time of the wma file in ms
*/
extern int			wma_dec_get_playtime(char *wmadec_workmem);				//get the play time of the wma file in ms

/*
* Function Name :  wma_dec_reset_offset
* Syntax : void wma_dec_reset_offset(char *wmadec_workmem);
* Purpose :  reset offset value which is to jump from ringbuffer
* Parameters : char *wmadec_workmem: allocated working memory pointer
* Return : none
*/
extern void			wma_dec_reset_offset(char *wmadec_workmem);				//reset offset value which is to jump from ringbuffer.

/*
* Function Name :  wma_dec_set_ri
* Syntax : void wma_dec_set_ri(char *wmadec_workmem, int ri);
* Purpose :  set read index of bit stream ring buffer
* Parameters : char *wmadec_workmem: allocated working memory pointer
*							 int ri: read index
* Return : none
*/
extern void			wma_dec_set_ri(char *wmadec_workmem, int ri);			//set ri pointer of ringbuffer.

/*
* Function Name :  wma_dec_get_offset
* Syntax : int wma_dec_get_offset(char *wmadec_workmem);
* Purpose :  get offset value of ring buffer
* Parameters : char *wmadec_workmem: allocated working memory pointer
* Return : offset of ring buffer
*/
extern int			wma_dec_get_offset(char *wmadec_workmem);				//get offset value of ringbuffer.

/*
* Function Name :  wma_dec_set_volume
* Syntax : void wma_dec_set_volume(char *wmadec_workmem, unsigned short volume);
* Purpose :  set volume
* Parameters : char *wmadec_workmem: allocated working memory pointer
*							 unsigned short volume: volume of a wma file
* Return : none
*/
extern void			wma_dec_set_volume(char *wmadec_workmem, unsigned short volume);//set EQ parameters of the wma file

// SUCCESS codes
#define WMA_OK						0x00000000
#define WMA_S_NEWPACKET				0x00000001
#define WMA_S_NO_MORE_FRAME			0x00000002
#define WMA_S_NO_MORE_SRCDATA		0x00000003
#define WMA_S_LOSTPACKET			0x00000004
#define WMA_S_CORRUPTDATA			0x00000005

// ERROR codes
#define WMA_E_FAIL					0x80004005
#define WMA_E_INVALIDARG			0x80070057
#define WMA_E_NOTSUPPORTED			0x80040000
#define WMA_E_NOTSUPPORTED_DRM		0x80040001
#define WMA_E_NOTSUPPORTED_CODEC    0x80040008
#define WMA_E_NOTSUPPORTED_VERSION	0x80040009
#define WMA_E_BROKEN_FRAME			0x80040002
#define WMA_E_DATANOTENOUGH			0x80040003//add by zgq
#define WMA_E_ONHOLD				0x80040004
#define WMA_E_NO_MORE_SRCDATA		0x80040005
#define WMA_E_WRONGSTATE			0x8004000A

#define WMA_E_BAD_PACKET_HEADER		0x80000011
#define WMA_E_NO_MORE_FRAMES		0x80000012
#define WMA_E_BAD_DRM_TYPE			0x80000013
#define WMA_E_INTERNAL				0x80000014
//#define WMA_E_NOMOREDATA_THISTIME	0x80000015
#define WMA_E_INVALIDHEADER			0x80000017
#define WMA_E_BUFFERTOOSMALL		0x80000018
#define WMA_E_WRONGVALUE			0x80000019


/* WAVE */
#define WAVE_OUT_INTERLEAVED

#define WAV_DEC_FRAMESIZE				2048
#define WAV_DEC_BITSTREAM_BUFFER_SIZE   4096
#define WAV_DEC_MEMORY_SIZE				100

//=======wave format tag====================
#define	WAVE_FORMAT_PCM				(0x0001)
#define	WAVE_FORMAT_ADPCM			(0x0002)
#define	WAVE_FORMAT_ALAW			(0x0006)
#define	WAVE_FORMAT_MULAW			(0x0007)
#define WAVE_FORMAT_IMA_ADPCM		(0x0011)

#define WAVE_FORMAT_A1800			(0x1000)
#define WAVE_FORMAT_MP3				(0x2000)
#define PCM_FORMAT_VR				(0x3000)
#define PCM_FORMAT_AUDIO_STREAM     (0x4000)
//========wave decoder error NO.====================
#define WAV_DEC_NO_ERROR    			0
#define WAV_DEC_INBUFFER_NOT_ENOUGH		0x80000001
#define WAV_DEC_RIFF_HEADER_NOT_FOUND	0x80000002
#define WAV_DEC_HEADER_NOT_FOUND		0x80000003
#define WAV_DEC_CHUNK_ALIGN_ERROR		0x80000004
#define WAV_DEC_DATA_CHUNK_NOT_FOUND	0x80000005
#define WAV_DEC_FMT_CHUNK_TOO_SHORT		0x80000006
#define WAV_DEC_CHANNEL_ERROR			0x80000007
#define WAV_DEC_BIT_DEPTH_ERROR			0x80000008
#define WAV_DEC_CHUNK_EXTEND_ERROR		0x80000009
#define WAV_DEC_MSADPCM_COEF_ERROR		0x8000000A
#define WAV_DEC_UNKNOWN_FORMAT			0x8000000B

int wav_dec_init(void *p_workmem, const unsigned char *p_bsbuf);
// p_bsbuf:      pointer to bit-stream buffer
// p_workmem:    pointer to working memory
// return value: 0 success

int wav_dec_parsing(void *p_workmem, int wi);//, WAVEFORMATEX *p_wave_format
// p_workmem:    pointer to working memory
// wi:           write index of bitstream ring buffer
// return value: 0 success
int wav_dec_set_param(void *p_workmem, const unsigned char *p_WaveFormatEx);
//this function is only used for AVI

void wav_dec_set_ring_buf_size(void *p_workmem, int size);

int wav_dec_run(void *p_workmem, short *p_pcmbuf, int wi);
// p_ pcmbuf:    pointer to PCM buffer
// p_workmem:    pointer to working memory
// wi:           write index of bitstream ring buffer
// return value:
//	positive : samples of PCM
//  zero:      not enough bitstream data
//  negtive:   error

int wav_dec_get_ri(void *p_workmem);
// p_workmem:    pointer to working memory
// return value: read index of bitstream ring buffer

int wav_dec_get_mem_block_size(void);
// return value: Wave Decoder Working memory size

int wav_dec_get_wFormatTag(void *p_workmem);
// p_workmem:    pointer to working memory
// return value: wave format tag

int wav_dec_get_nChannels(void *p_workmem);
// p_workmem:    pointer to working memory
// return value: number of channels

int wav_dec_get_SampleRate(void *p_workmem);
// p_workmem:    pointer to working memory
// return value: sample rate

int wav_dec_get_nAvgBytesPerSec(void *p_workmem);
// p_workmem:    pointer to working memory
// return value: average bytes per second

int wav_dec_get_nBlockAlign(void *p_workmem);
// p_workmem:    pointer to working memory
// return value: block align

int wav_dec_get_wBitsPerSample(void *p_workmem);
// p_workmem:    pointer to working memory
// return value: bits per sample
const INT8U * wav_dvr_get_version(void);

//---------------------------------------------------------------------------
/* WAVE Encode*/
#ifndef __WAV_ENC_H__
#define __WAV_ENC_H__

//=======Constant Definition================
#define WAV_ENC_MEMORY_BLOCK_SIZE		40
#define DVRWAVE_FRAME_SIZE				512

//========wave decoder error NO.==================
#define WAV_ENC_NO_ERROR    			0
#define WAV_ENC_UNKNOWN_FORMAT			0x80000100
#define WAV_ENC_CHANNEL_ERROR			0x80000101

int wav_enc_init(void *p_workmem);

int wav_enc_run(void *p_workmem,short *p_pcmbuf,unsigned char *p_bsbuf);

int wav_enc_get_header(void *p_workmem, void *p_header,int length, int numsamples);

int wav_enc_get_mem_block_size(void);

int wav_enc_get_SamplePerFrame(void *p_workmem);

int wav_enc_get_BytePerPackage(void *p_workmem);

int wav_enc_get_HeaderLength(void *p_workmem);

int wav_enc_Set_Parameter(void *p_workmem, int Channel,int SampleRate,int FormatTag);

#endif //__WAV_ENC_H__

//---------------------------------------------------------------------------
//A1800 decode
#ifndef __A1800DEC_H__
#define __A1800DEC_H__

#define A1800DEC_MEMORY_BLOCK_SIZE		5824//1676

#define A18_DEC_FRAMESIZE        320
#define A18_DEC_BITSTREAM_BUFFER_SIZE 4096

#define A18_OK						0x00000001
#define A18_E_NO_MORE_SRCDATA		0x80040005

extern int  A18_dec_SetRingBufferSize(void *obj, int size);
extern int  a1800dec_run(void *obj, int write_index, short * pcm_out);
extern int  a1800dec_init(void *obj, const unsigned char* bs_buf);
extern int  a1800dec_parsing(void *obj, int write_index);
extern int  a1800dec_read_index(void *obj);
extern int  a1800dec_GetMemoryBlockSize(void);
extern int  a1800dec_errno(void *obj);

extern const char* A18_dec_get_version(void);
extern int  A18_dec_get_bitrate(void *obj);
extern int  A18_dec_get_samplerate(void *obj);
extern int	A18_dec_get_channel(void *obj);
extern int	A18_dec_get_bitspersample(void *obj);
#endif //__A1800DEC_H__

//---------------------------------------------------------------------------
/* a1800 encode */
#ifndef __A1800ENC_H__
#define __A1800ENC_H__

#define	A18_ENC_FRAMESIZE			320		// input pcm size per frame
#define	A18_ENC_MEMORY_SIZE			5784	//

#define A18_ENC_NO_ERROR			0
#define A18_E_MODE_ERR				0x80000004

extern int A18_enc_run(unsigned char *p_workmem, const short *p_pcmbuf, unsigned char *p_bsbuf);
extern int A18_enc_init(unsigned char *p_workmem);
extern int A18_enc_get_BitRate(unsigned char *p_workmem);
extern int A18_enc_get_PackageSize(unsigned char *p_workmem);
extern int A18_enc_get_errno(char *A18enc_workmem);
extern const char* A18_enc_get_version(void);
extern int A18_enc_get_mem_block_size(void);
extern void A18_enc_set_BitRate(unsigned char *p_workmem, int BitRate);
#endif //!__A1800ENC_H__

//---------------------------------------------------------------------------
//A1600
#ifndef __A16_DEC_H__
#define __A16_DEC_H__

#define A16_DEC_FRAMESIZE        128
#define A16_DEC_MEMORY_SIZE      164
#define A16_DEC_BITSTREAM_BUFFER_SIZE 1024

#define     A16_IS_NOT_AT_FILE_END			0		//not arrive at the file end.must be no-zero.
#define     A16_IS_AT_FILE_END				1		//arrive at the file end.must be zero.

#define A16_OK						0
#define A16_E_NO_MORE_SRCDATA			0x80000000
#define A16_E_READ_IN_BUFFER			0x80000001
#define A16_CODE_FILE_FORMAT_ERR		0x80000002
#define A16_E_FILE_END				0x80000003
#define A16_E_MODE_ERR				0x80000004

extern int			A16_dec_run(char *p_workmem, short *p_pcmbuf, int wi);
extern int			A16_dec_init(char* A16dec_workmem, unsigned char* bs_buf);
extern int			A16_dec_parsing(char *A16dec_workmem,int wi);
extern int			A16_dec_get_ri(char *A16dec_workmem);
extern int			A16_dec_get_errno(char *A16dec_workmem);
extern const char * A16_dec_get_version(void);
extern int			A16_dec_get_mem_block_size(void);
extern int			A16_dec_get_bitrate(char *A16dec_workmem);
extern int			A16_dec_get_samplerate(char *A16dec_workmem);
extern int			A16_dec_get_channel(char *A16dec_workmem);
extern int			A16_dec_get_bitspersample(char *A16dec_workmem);
extern void			A16_dec_set_AtFileEnd(char *A16dec_workmem);
extern int			A16_dec_get_FileLen(char *A16dec_workmem);

#endif //!__A16_DEC_H__


//-----------------------------------------------------------------------added by Bruce, 2008/09/26
//A6400
#ifndef __a6400_dec_h__
#define __a6400_dec_h__

/////////////////////////////////////////////////////////////////////////////
//		Constant Definition
/////////////////////////////////////////////////////////////////////////////
#define A6400_DEC_FRAMESIZE					1152	// ???
#define A6400_DEC_BITSTREAM_BUFFER_SIZE   	4096    // size in bytes
#define A6400_DEC_MEMORY_SIZE 			13512
#define A6400_DECODE_RAM     			7176

/////////////////////////////////////////////////////////////////////////////
//		Error Code
/////////////////////////////////////////////////////////////////////////////
#define A6400_DEC_ERR_NONE				0x00000000	/* no error */

#define A6400_DEC_ERR_BUFLEN	   	   	0x80000001	/* input buffer too small (or EOF) */
#define A6400_DEC_ERR_BUFPTR	   	   	0x80000002	/* invalid (null) buffer pointer */

#define A6400_DEC_ERR_NOMEM	   	   		0x80000031	/* not enough memory */

#define A6400_DEC_ERR_LOSTSYNC	   		0x80000101	/* lost synchronization */
#define A6400_DEC_ERR_BADLAYER	   		0x80000102	/* reserved header layer value */
#define A6400_DEC_ERR_BADBITRATE	   	0x80000103	/* forbidden bitrate value */
#define A6400_DEC_ERR_BADSAMPLERATE  	0x80000104	/* reserved sample frequency value */
#define A6400_DEC_ERR_BADEMPHASIS	   	0x80000105	/* reserved emphasis value */
#define A6400_DEC_ERR_BADMPEGID			0x80000106	//for error mpegid add by zgq on 20080508

#define A6400_DEC_ERR_BADCRC	   	   	0x80000201	/* CRC check failed */
#define A6400_DEC_ERR_BADBITALLOC	   	0x80000211	/* forbidden bit allocation value */
#define A6400_DEC_ERR_BADSCALEFACTOR  	0x80000221	/* bad scalefactor index */
#define A6400_DEC_ERR_BADMODE         	0x80000222	/* bad bitrate/mode combination */
#define A6400_DEC_ERR_BADFRAMELEN	    0x80000231	/* bad frame length */
#define A6400_DEC_ERR_BADBIGVALUES    	0x80000232	/* bad big_values count */
#define A6400_DEC_ERR_BADBLOCKTYPE    	0x80000233	/* reserved block_type */
#define A6400_DEC_ERR_BADSCFSI	    	0x80000234	/* bad scalefactor selection info */
#define A6400_DEC_ERR_BADDATAPTR	    0x80000235	/* bad main_data_begin pointer */
#define A6400_DEC_ERR_BADPART3LEN	    0x80000236	/* bad audio data length */
#define A6400_DEC_ERR_BADHUFFTABLE    	0x80000237	/* bad Huffman table select */
#define A6400_DEC_ERR_BADHUFFDATA	    0x80000238	/* Huffman data overrun */
#define A6400_DEC_ERR_BADSTEREO	    	0x80000239	/* incompatible block_type for JS */

/////////////////////////////////////////////////////////////////////////////
//		Function Definition
/////////////////////////////////////////////////////////////////////////////


// A6400 Decoder Initial
int a6400_dec_init(char *p_workmem, unsigned char *p_bsbuf, char *ram);


// A6400 header parsing
int a6400_dec_parsing(char *p_workmem, unsigned int wi);

// A6400 Decoder
int a6400_dec_run(char *p_workmem, short *p_pcmbuf, unsigned int wi);


// Get Read Index
int a6400_dec_get_ri(char *p_workmem);
// p_workmem:    pointer to working memory
// return value: read index of bitstream ring buffer

// Set Read Index
void a6400_dec_set_ri(char *a6400dec_workmem, int ri);

// Set bitstream buffer size
void a6400_dec_set_bs_buf_size(char *a6400dec_workmem, int bs_buf_size);

// Get mpeg id
const char *a6400_dec_get_mpegid(char *p_workmem);

int a6400_dec_get_mem_block_size (void);
// return value: A6400 Decoder Working memory size

// return error number.
int a6400_dec_get_errno(char *p_workmem);

// return layer.
int a6400_dec_get_layer(char *p_workmem);

// return channel.
int a6400_dec_get_channel(char *p_workmem);

// return bitrate in kbps.
int a6400_dec_get_bitrate(char *p_workmem);

// return sampling rate in Hz.
int a6400_dec_get_samplerate(char *p_workmem);

// get library version
const unsigned char * a6400_dec_get_version(void);

// set EQ
void a6400_dec_set_EQ(unsigned char *a6400dec_workmem, unsigned short *EQ_table);


// set volume gain, Q.14 (1 = 0x4000)
extern void a6400_dec_set_volume(char *a6400dec_workmem, unsigned short vol);


#endif  // __a6400_dec_h__

//-----------------------------------------------------------------------added by Bruce, 2008/09/26
//S880
#ifndef __S880_DEC_H__
#define __S880_DEC_H__

#define L_FRAME16k   320                   /* Frame size at 16kHz                        */

#define S880_DEC_FRAMESIZE        L_FRAME16k
//#define S880_DEC_MEMORY_SIZE      2072
//#define S880_DEC_MEMORY_SIZE      (2072+1296)		//wenli, 2008.9.12
#define S880_DEC_MEMORY_SIZE      (2072+1296+128+184)		//wenli, 2008.9.18

#define S880_DEC_BITSTREAM_BUFFER_SIZE 1024

#define     S880_IS_NOT_AT_FILE_END			0		//not arrive at the file end.must be no-zero.
#define     S880_IS_AT_FILE_END				1		//arrive at the file end.must be zero.

#define S880_OK							0
#define S880_E_NO_MORE_SRCDATA			0x80000000
#define S880_E_READ_IN_BUFFER			0x80000001
#define S880_CODE_FILE_FORMAT_ERR		0x80000002
#define S880_E_FILE_END					0x80000003
#define S880_ERR_INVALID_MODE 			0x80000004

extern int			S880_dec_run(char *p_workmem, short *p_pcmbuf, int wi);
extern int			S880_dec_init(char* S880dec_workmem, unsigned char* bs_buf);
extern int			S880_dec_parsing(char *S880dec_workmem,int wi);
extern int			S880_dec_get_ri(char *S880dec_workmem);
extern int			S880_dec_get_errno(char *S880dec_workmem);
extern const char * S880_dec_get_version(void);
extern int			S880_dec_get_mem_block_size(void);
extern int			S880_dec_get_bitrate(char *S880dec_workmem);
extern int			S880_dec_get_samplerate(char *S880dec_workmem);
extern int			S880_dec_get_channel(char *S880dec_workmem);
extern int			S880_dec_get_bitspersample(char *S880dec_workmem);
extern void			S880_dec_set_AtFileEnd(char *S880dec_workmem);
extern int			S880_dec_get_FileLen(char *S880dec_workmem);

#endif //!__S880_DEC_H__
//==========================================================================
#ifndef __NEAACDEC_H__
#define __NEAACDEC_H__


/* A decode call can eat up to FAAD_MIN_STREAMSIZE bytes per decoded channel,
   so at least so much bytes per channel should be available in this stream */
#define FAAD_MIN_STREAMSIZE	 768 /* 6144 bits/channel */
#define AAC_DEC_FRAMESIZE	1024
#define AAC_DEC_BITSTREAM_BUFFER_SIZE 4096

//#define SUPPORT_5_1_CHANNELS

#ifdef SUPPORT_5_1_CHANNELS
#define MAX_CHANNELS		 6
#else
#define MAX_CHANNELS		 2
#endif


#ifdef SUPPORT_5_1_CHANNELS
#define	AAC_DEC_MEMORY_BLOCK_SIZE	20816//24956
#else
#define	AAC_DEC_MEMORY_BLOCK_SIZE	12672//16700
#endif



// ***************************************************************************
#define AAC_OK														0x0


#define UNSUPPORTED_FILE_FORMAT_MP4									0x80000001
#define NOT_MONO_OR_STEREO											0x80000002

#define NOT_LC_OBJECT_TYPE											0x80000011
#define UNABLE_TO_FIND_ADTS_SYNCWORD								0x80000012

#define GAIN_CONTROL_NOT_YET_IMPLEMENTED                            0x80000021
#define PULSE_CODING_NOT_ALLOWED_IN_SHORT_BLOCKS                    0x80000022
#define SCALEFACTOR_OUT_OF_RANGE                                    0x80000023
#define CHANNEL_COUPLING_NOT_YET_IMPLEMENTED                        0x80000024
#define ERROR_DECODING_HUFFMAN_CODEWORD					            0x80000025
#define NON_EXISTENT_HUFFMAN_CODEBOOK_NUMBER_FOUND                  0x80000026
#define INVALID_NUMBER_OF_CHANNELS                                  0x80000027
#define MAXIMUM_NUMBER_OF_BITSTREAM_ELEMENTS_EXCEEDED               0x80000028
#define INPUT_DATA_BUFFER_TOO_SMALL                                 0x80000029
#define ARRAY_INDEX_OUT_OF_RANGE                                    0x8000002A
#define MAXIMUM_NUMBER_OF_SCALEFACTOR_BANDS_EXCEEDED                0x8000002B
#define QUANTISED_VALUE_OUT_OF_RANGE								0x8000002C
#define UNEXPECTED_CHANNEL_CONFIGURATION_CHANGE                     0x8000002D
#define ERROR_IN_PROGRAM_CONFIG_ELEMENT                             0x8000002E
#define PCE_SHALL_BE_THE_FIRST_ELEMENT_IN_A_FRAME                   0x8000002F
#define BITSTREAM_VALUE_NOT_ALLOWED_BY_SPECIFICATION                0x80000030
// ***************************************************************************
/* object types for AAC */
#define MAIN       1
#define LC         2
#define SSR        3
#define LTP        4
#define HE_AAC     5
#define ER_LC     17
#define ER_LTP    19
#define LD        23
#define DRM_ER_LC 27 /* special object type for DRM */
// ***************************************************************************



int aac_dec_init(void *pWorkMem, int downMatrix, const unsigned char *bs_buf);

int aac_dec_parsing(void *pWorkMem, int wi);

/*static const uint32_t sample_rates[] =
{
	96000, 88200, 64000, 48000, 44100, 32000,
	24000, 22050, 16000, 12000, 11025,  8000
};*/

int aac_dec_set_param(void *pWorkMem,
					  unsigned char objectTypeIndex,			// object type (only support LC profile)
					  unsigned char samplingFrequencyIndex,		// sample rate index (refer to sample_rates[])
					  unsigned char channelsConfiguration);		// channels

int aac_dec_run(void *pWorkMem, int wi, short *pcm_out);

unsigned long aac_dec_get_samplerate(void *pWorkMem);
unsigned char aac_dec_get_channel(void *pWorkMem);
int	aac_dec_get_bitspersample(void *pWorkMem);

#ifdef SUPPORT_5_1_CHANNELS
int aac_dec_if_lfe_channel_exists(void *pWorkMem);
#endif

int aac_dec_get_mem_block_size(void);

int aac_dec_get_read_index(void *pWorkMem);

int aac_dec_get_errno(void *pWorkMem);
const char *aac_dec_get_version(void);
int aac_dec_SetRingBufferSize(void *pWorkMem, int size);
void aac_dec_set_ri(void *pWorkMem, int ri_addr);

#endif

//-----------------------------------------------------------------------
#ifndef __oggvorbis_dec_h__
#define __oggvorbis_dec_h__

#define overlap_pcmbuffer

// Constant Definition //
#define OGGVORBIS_DEC_FRAMESIZE					2048
#define OGGVORBIS_DEC_BITSTREAM_BUFFER_SIZE   	4096
#define OGGVORBIS_DEC_MEMORY_SIZE 				65992

// error code //
enum vorbis_return_value {
	OGGVORBIS_MORE_DATA   =  0,
	OGGVORBIS_INIT_OK     =  0,
	OGGVORBIS_PARSING_OK  =  1,
	OGGVORBIS_FILE_END    = -1,
	OGGVORBIS_FATAL_ERROR = -2
};

enum error_number {
	OGG_ERR_OK         =  0,
	ERR_ID_HEADER      = -1,
	ERR_COMMENT_HEADER = -2,
	ERR_SETUP_HEADER   = -3,
	ERR_PACKETBUFFER_NOTENOUGH = -4,
	ERR_NOT_SUPPORT_WIN_SIZE = -5
};

// Function Definition //
//========================================================
// Function Name : oggvorbis_dec_init
// Syntax : int oggvorbis_dec_init(void *p_workmem, const char *Ring, int RingSize, int RingRI);
// Purpose :  initial kernel
// Parameters : void *p_workmem : allocated working memory
//              char *Ring : pointer to ring-buffer
//              int RingSize : size of ring-buffer
//              int RingRI : Read-Index of ring-buffer
// Return : "0" if init successfully
//========================================================
int oggvorbis_dec_init(void *p_workmem, const char *Ring, int RingSize, int RingRI);

//========================================================
// Function Name : oggvorbis_dec_set_ring_buffer
// Syntax : void oggvorbis_dec_set_ring_buffer(void *p_workmem, const char *Ring, int RingSize, int RingRI);
// Purpose : set ring-buffer information into kernel
// Parameters : void *p_workmem : allocated working memory
//              char *Ring : pointer to ring-buffer
//              int RingSize : size of ring-buffer
//              int RingRI : Read-Index of ring-buffer
// Return : none
//========================================================
void oggvorbis_dec_set_ring_buffer(void *p_workmem, const char *Ring, int RingSize, int RingRI);

//========================================================
// Function Name : oggvorbis_dec_parsing
// Syntax : int oggvorbis_dec_parsing(char *p_workmem, unsigned int wi);
// Purpose : parsing header information
// Parameters : char *p_workmem: allocated working memory
//              int wi: write index of bitstream buffer
// Return : "0" for parsing successfully
//          "others value" for parsing fail(Check error code)
//========================================================
int oggvorbis_dec_parsing(void *p_workmem, int wi);

//========================================================
// Function Name : oggvorbis_dec_run
// Syntax : int oggvorbis_dec_run(char *p_workmem, short *p_pcmbuf, int wi);
// Purpose : main decode process
// Parameters : char *p_workmem: allocated working memory
//              short *p_pcmbuf: input pcm buffer address
//              int wi: write index of bitstream buffer
// Return : "Positive" : number of decoded PCM sample
//          "others value" : decode fail(Check error code)
//========================================================
int oggvorbis_dec_run(void *p_workmem, short *p_pcmbuf, int wi);

//========================================================
// Function Name : oggvorbis_dec_get_ri
// Syntax : int oggvorbis_dec_get_ri(char *p_workmem);
// Purpose :  get read index of bitstream ring buffer
// Parameters : char *p_workmem: allocated working memory pointer
// Return : read index of bit stream ring buffer
//========================================================
int oggvorbis_dec_get_ri(void *p_workmem);

//========================================================
// Function Name : oggvorbis_dec_get_mem_block_size
// Syntax : int oggvorbis_dec_get_mem_block_size (void);
// Purpose : Get the size of Working memory
// Parameters : none
// Return : size of Working memory
//========================================================
int oggvorbis_dec_get_mem_block_size (void);

//========================================================
// Function Name : oggvorbis_dec_get_errno
// Syntax : int oggvorbis_dec_get_errno(void *p_workmem);
// Purpose : get error number
// Parameters : void *p_workmem: allocated working memory pointer
// Return : return error code
//========================================================
int oggvorbis_dec_get_errno(void *p_workmem);

//========================================================
// Function Name : oggvorbis_dec_get_channel
// Syntax : int oggvorbis_dec_get_channel(void *p_workmem);
// Purpose :  get channel
// Parameters : void *p_workmem: allocated working memory pointer
// Return : 1: mono; 2: stereo
//========================================================
int oggvorbis_dec_get_channel(void *p_workmem);

//========================================================
// Function Name : oggvorbis_dec_get_bitrate
// Syntax : int oggvorbis_dec_get_bitrate(void *p_workmem);
// Purpose :  get bit rate
// Parameters : void *p_workmem: allocated working memory pointer
// Return : bit rate in bps
//========================================================
int oggvorbis_dec_get_bitrate(void *p_workmem);

//========================================================
// Function Name : oggvorbis_dec_get_samplerate
// Syntax : int oggvorbis_dec_get_samplerate(void *p_workmem);
// Purpose :  get sample rate
// Parameters : void *p_workmem: allocated working memory pointer
// Return : sample rate in Hz
//========================================================
int oggvorbis_dec_get_samplerate(void *p_workmem);

//========================================================
// Function Name : oggvorbis_dec_get_version
// Syntax : unsigned char * oggvorbis_dec_get_version(void);
// Purpose : get version
// Parameters : none
// Return : version of library
//========================================================
const char * oggvorbis_dec_get_version(void);

//========================================================
// Function Name : oggvorbis_dec_get_sample_stamp
// Syntax : int oggvorbis_dec_get_sample_stamp(void *oggvorbis_instance);
// Purpose : get sample stamp
// Parameters : none
// Return : sample stamp
//========================================================
int oggvorbis_dec_get_sample_stamp(void *oggvorbis_instance);

#endif  // __oggvorbis_dec_h__

//==========================================================================
#ifndef __MP3ENC_H__
#define __MP3ENC_H__

#define	MP3_ENC_WORKMEM_SIZE  	31704

// layer3.c //
extern int mp3enc_init(
	void *pWorkMem,
	int nChannels,
	int nSamplesPreSec,
	int nKBitsPreSec,
	int Copyright,
	char *Ring,
	int RingBufSize,
	int RingWI);

extern int mp3enc_encframe(void *pWorkMem, const short *PCM);
extern int mp3enc_end(void *pWorkMem);

#define MP3ENC_ERR_INVALID_CHANNEL		0x80000001
#define MP3ENC_ERR_INVALID_SAMPLERATE	0x80000002
#define MP3ENC_ERR_INVALID_BITRATE		0x80000003

const char *mp3enc_GetErrString(int ErrCode);
const char *mp3enc_GetVersion(void);
int mp3enc_GetWorkMemSize(void);

#endif	// __MP3ENC_H__
//==========================================================================
#ifndef __CONSTANT_PITCH_H__
#define __CONSTANT_PITCH_H__

// Initial Link and Unlink functions
int ConstantPitch_Link(
	void *pWorkMem,
	void *pFrontEndWorkMem,
	int (*GetOutput)(void *, short *, int),
	int fs,
	int ch,
	int OptionFlag);

void ConstantPitch_Unlink(void *pWorkMem);

// version
const char *ConstantPitch_GetVersion(void);

// functions that used for memory allocated by ConstantPitch
void *ConstantPitch_Create(int InFrameSize, int fs, int ch, int none);
void ConstantPitch_Del(void *pWorkMem);

// set functions
extern int ConstantPitch_SetParam(void *pWorkMem, int pitch_idx, int none);

// get output function
int ConstantPitch_GetOutput(void *pWorkMem, short *DstData, int Len);
//int Robot_GetOutput(void *pWorkMem, short *DstData, int Len);

void ConstantPitch_Del(void *pWorkMem);

// get functions
int ConstantPitch_GetSampleRate(void *pWorkMem);
int ConstantPitch_GetChannel(void *pWorkMem);

#endif // __CONSTANT_PITCH_H__
//==========================================================================
#ifndef __ECHO_H__
#define __ECHO_H__

// version
const char *Echo_GetVersion(void);

// functions that used for memory allocated by Echo
void *Echo_Create(int InFrameSize, int fs, int ch, int OptFlag);
void Echo_Del(void *pWorkMem);

// functions that used for memory allocated by User
int Echo_GetMemBlockSize(int InFrameSize, int fs, int ch, int OptFlag);
void Echo_Initial(void *pWorkMem, int InFrameSize, int fs, int ch, int OptFlag);

// Initial Link and Unlink functions
int Echo_Link(
	void *pWorkMem,
	void *pFrontEndWorkMem,
	int (*GetOutput)(void *, short *, int),
	int fs,
	int ch,
	int OptionFlag);

// opt:
#define OPTFLAG_LEFT_CH_ONLY	1
// This flag only take effect if ch == 2
// If this flag is set, Echo module will convert 2-ch inuput to 1-ch(Left only) output

void Echo_Unlink(void *pWorkMem);

// set functions
extern int Echo_SetParam(void *pWorkMem, int delay_len, int weight_idx);

// get output function
int Echo_GetOutput(void *pWorkMem, short *DstData, int Len);

// get functions
int Echo_GetSampleRate(void *pWorkMem);
int Echo_GetChannel(void *pWorkMem);

#endif // __ECHO_H__
//==========================================================================
#ifndef __VOICE_CHANGER_H__
#define __VOICE_CHANGER_H__

// version
const char *VoiceChanger_GetVersion(void);

// functions that used for memory allocated by VoiceChanger
void *VoiceChanger_Create(int InFrameSize, int fs, int ch, int OptFlag);

void VoiceChanger_Del(void *pWorkMem);

// functions that used for memory allocated by User
int VoiceChanger_GetMemBlockSize(int InFrameSize, int fs, int ch, int OptFlag);
void VoiceChanger_Initial(void *pWorkMem, int InFrameSize, int fs, int ch, int OptFlag);

// Initial Link and Unlink functions
void VoiceChanger_Link(
	void *pWorkMem,
	void *pFrontEndWorkMem,
	int (*GetOutput)(void *, short *, int),
	int fs,
	int ch,
	int OptionFlag);

// opt:
#define OPTFLAG_LEFT_CH_ONLY	1
// This flag only take effect if ch == 2
// If this flag is set, VoiceChanger module will convert 2-ch inuput to 1-ch(Left only) output

void VoiceChanger_Unlink(void *pWorkMem);

// set functions
extern void VoiceChanger_SetParam(void *pWorkMem, int speed, int pitch);
// speed : can be 0~24. Default 12, means using original speed as output.
// This setting represents the following times of original speed:
//	0.5x,	0.5x,	0.562x,	0.625x,	0.625x,	0.687x,	0.687x,	0.75x,	0.812x,	0.812x,	0.875x,	0.938x,	1.0x,
//	1.063x,	1.125x,	1.188x,	1.25x,	1.313x,	1.375x,	1.5x,	1.563x,	1.688x,	1.813x,	1.877x,	2.0x,
//
// pitch : can be 0~24. Default 12, means using original pitch as output.
// This setting represents the following times of original pitch:
//	0.5x,	0.5x,	0.562x,	0.625x,	0.625x,	0.687x,	0.687x,	0.75x,	0.812x,	0.812x,	0.875x,	0.938x,	1.0x,
//	1.063x,	1.125x,	1.188x,	1.25x,	1.313x,	1.375x,	1.5x,	1.563x,	1.688x,	1.813x,	1.877x,	2.0x,
//

// get output function
int VoiceChanger_GetOutput(void *pWorkMem, short *DstData, int Len);

// get functions
int VoiceChanger_GetSampleRate(void *pWorkMem);
int VoiceChanger_GetChannel(void *pWorkMem);

#endif // __VOICE_CHANGER_H__
//==========================================================================
#ifndef __unsample_api_h__
#define __unsample_api_h__

#define OPTION_FLAG_LEFT_CH_ONLY	1  // only working when ch=2 and upsample left channel

const char *UpSample_GetVersion(void);

// functions for allocate memory by user
unsigned int UpSample_GetMemSize(int decfrsize);
void UpSample_Init(void *pWorkMem, int decfrsize);									// added by Jacky Lu

// functions for allocate memory by Upsample Library itself
void *UpSample_Create(int decfrsize);
void UpSample_Del(void *pWorkMem);

// functions for Audio-Source-Interface
void UpSample_Link(void *pWorkMem, void *FrontEnd, int (*pfnGetData)(void *, short *, int), int fs, int ch, int factor, int OptionFlag);
void UpSample_Unlink(void *pWorkMem);
int UpSample_GetSampleRate(void *pWorkMem);
int UpSample_GetChannel(void *pWorkMem);

int UpSample_GetOutput(void *pWorkMem, short *DstData, int Len); // added by Jacky Lu

#endif
//==========================================================================
#ifndef __downsample_api_h__
#define __downsample_api_h__

const char *DownSample_get_version(void);

int DownSample_GetMemSize(int MaxEncFrameSize, int MaxDownSampleFactor);
void DownSample_Init(void *pWorkMem, int MaxEncFrameSize, int MaxDownSampleFactor);

void *DownSample_Create(int MaxEncFrameSize, int MaxDownSampleFactor);
void DownSample_Del(void *pWorkMem);

void DownSample_Link(
	void *pWorkMem,
	void *pBackEnd,
	int (*pBackEnd_PutData)(void *, const short*, int),
	int fs,
	int channel,
	int factor);

int DownSample_PutData(void *pWorkMem, const short *InData, int InLen);

int DownSample_GetSampleRate(void *_pWorkMem);
int DownSample_GetChannel(void *pWorkMem);
#endif
//==========================================================================
#ifndef __LPF_H__
#define __LPF_H__

#define Max_LoopCnt 10  //最大階數設定
extern void LPF_init(long coef_frq,short coef_loop);
extern INT16U LPF_process(INT16U FilterBuf);
#endif // __LPF_H__

// File System
//FS driver
INT32S NAND_Initial(void);
INT32S NAND_Uninitial(void);
INT32S NAND_APP_Initial(void);
INT32S NAND_APP_Uninitial(void);

#define FAT16_Type	        		0x01
#define FAT32_Type	        		0x02
#define FAT12_Type	        		0x03
#define EXFAT_Type					0x04
#define FORCE_FAT32_Type	        0x12
#define FORCE_FAT16_Type	        0x11
#define FORCE_EXFAT_Type			0x14

/*-----------------  seek flags  ------------------*/
#define SEEK_SET     	0 		/* offset from beginning of the file*/
#define SEEK_CUR     	1 		/* offset from current location     */
#define SEEK_END     	2 		/* offset from eof  */

/***************** open flags (the 2nd parameter)**********************/
#define O_RDONLY        0x0000
#define O_WRONLY        0x0001
#define O_RDWR          0x0002
#define O_ACCMODE       0x0003

#define O_TRUNC         0x0200 	/*    both */
#define O_CREAT         0x0400
#define O_EXCL		    0x4000	/* not fcntl */

/* File attribute constants for _findfirst() */
#define _A_NORMAL       0x00    /* Normal file - No read/write restrictions */
#define _A_RDONLY       0x01    /* Read only file */
#define _A_HIDDEN       0x02    /* Hidden file */
#define _A_SYSTEM       0x04    /* System file */
#define _A_SUBDIR       0x10    /* Subdirectory */
#define _A_ARCH         0x20    /* Archive file */

/* FAT file system attribute bits                               */
#define D_NORMAL        0       /* normal                       */
#define D_RDONLY        0x01    /* read-only file               */
#define D_HIDDEN        0x02    /* hidden                       */
#define D_SYSTEM        0x04    /* system                       */
#define D_VOLID         0x08    /* volume id                    */
#define D_DIR           0x10    /* subdir                       */
#define D_ARCHIVE       0x20    /* archive bit                  */

#define D_FILE			(0x40)	/* all attribute but D_DIR		*/
#define D_FILE_1		(0x80)	/* contain D_NORMAL,D_RDONLY,D_ARCHIVE */
#define D_ALL (D_FILE | D_RDONLY | D_HIDDEN | D_SYSTEM | D_DIR | D_ARCHIVE)

#define UNI_GBK			0
#define UNI_BIG5		1
#define UNI_SJIS		2

#define UNI_ENGLISH		0x8003
#define UNI_ARABIC		0x8004
#define UNI_UNICODE		0x8100

// file system error code
/* Internal system error returns                                */
#define SUCCESS         0       /* Function was successful      */
#define DE_INVLDFUNC    -1      /* Invalid function number      */
#define DE_FILENOTFND   -2      /* File not found               */
#define DE_PATHNOTFND   -3      /* Path not found               */
#define DE_TOOMANY      -4      /* Too many open files          */
#define DE_ACCESS       -5      /* Access denied                */
#define DE_INVLDHNDL    -6      /* Invalid handle               */
#define DE_MCBDESTRY    -7      /* Memory control blocks shot   */
#define DE_NOMEM        -8      /* Insufficient memory          */
#define DE_INVLDMCB     -9      /* Invalid memory control block */
#define DE_INVLDENV     -10     /* Invalid enviornement         */
#define DE_INVLDFMT     -11     /* Invalid format               */
#define DE_INVLDACC     -12     /* Invalid access               */
#define DE_INVLDDATA    -13     /* Invalid data                 */
#define DE_INVLDDRV     -15     /* Invalid drive                */
#define DE_RMVCUDIR     -16     /* Attempt remove current dir   */
#define DE_DEVICE       -17     /* Not same device              */
#define DE_MAX_FILE_NUM       -18     /* No more files                */
#define DE_WRTPRTCT     -19     /* No more files                */
#define DE_BLKINVLD     -20     /* invalid block                */
#define DE_INVLDBUF     -24     /* invalid buffer size, ext fnc */
#define DE_SEEK         -25     /* error on file seek           */
#define DE_HNDLDSKFULL  -28     /* handle disk full (?)         */
#define DE_INVLDPARM    -87     /* invalid parameter			*/
#define DE_UNITABERR    -88     /* unitab error					*/
#define DE_TOOMANYFILES	-89		/* to many files				*/

#define DE_DEADLOCK	-36
#define DE_LOCK		-39

#define DE_INVLDCDFILE	-48		/* invalid cd file name */
#define DE_NOTEMPTY		-49		/* DIR NOT EMPTY */
#define DE_ISDIR		-50		/* Is a directory name          */
#define DE_FILEEXISTS   -80		/* File exists                  */
#define DE_DEVICEBUSY	-90
#define DE_NAMETOOLONG	-100	/* specified path name too long */
#define DE_FILENAMEINVALID -110	/* Invalid */


#define LFN_FLAG			1
#define WITHFAT32			1
#define WITHFAT12			1
#define WITHEXFAT           0

struct stat_t
{
	INT16U	st_mode;
  #if WITHEXFAT == 1
	INT64S	st_size;
  #else
    INT32S	st_size;
  #endif
	INT32U	st_mtime;
};

struct _diskfree_t {
	INT32U total_clusters;
	INT32U avail_clusters;
	INT32U sectors_per_cluster;
	INT32U bytes_per_sector;
};

struct deviceinfo 	 {
	INT8S device_name[16]; 		 // device name
	INT8S device_enable; 			 // device enable status
	INT8S device_typeFAT; 		 // device FAT type
	INT32U device_availspace; 	 // device available space
	INT32U device_capacity; 		 // device capacity
};

// data structure for _setftime()
struct timesbuf 	 {
	INT16U modtime;
	INT16U moddate;
	INT16U accdate;
};

struct f_info {
	INT8U	f_attrib;
	INT16U	f_time;
	INT16U	f_date;
  #if WITHEXFAT == 1
	INT64U	f_size;
  #else
	INT32U	f_size;
  #endif
	INT16U	entry;
	INT8S	f_name[256];
	INT8S	f_short_name[8 + 3 + 1];
};

typedef struct {
	INT32U  f_entry;
	INT16U  f_offset;
	INT8S   f_dsk;
	INT8S	f_is_root;		// to differentiate the root folder and the first folder in root folder, in disk with FAT16/FAT12 format
} f_pos, *f_ppos;

struct sfn_info {
    INT8U   f_attrib;
	INT16U  f_time;
	INT16U  f_date;
#if WITHEXFAT == 1
	INT64U	f_size;
#else
	INT32U	f_size;
#endif
    INT8S    f_name[9];
    INT8S    f_extname[4];
    f_pos	f_pos;
};

struct nls_table {
	CHAR			*charset;
	INT16U			Status;
	INT16S			(*init_nls)(void);
	INT16S			(*exit_nls)(void);
	INT16U			(*uni2char)(INT16U uni);
	INT16U			(*char2uni)(INT8U **rawstring);
};

typedef struct
{
	f_pos		cur_pos;
	f_pos		cur_dir_pos;
	INT16U		level;
	INT16S		find_begin_file;
	INT16S		find_begin_dir;
	INT16U		index;

	INT32U		sfn_cluster;
	INT32U		sfn_cluster_offset;

	INT8U		dir_change_flag;
	INT8U		root_dir_flag;			// if the root folder have found the file, this flag is setted
	INT16U		dir_offset[16];
	INT32U		cluster[16];
	INT32U	 	cluster_offset[16];
#if WITHEXFAT == 1
	INT32U 		FatherClu[16];			// 羶衄. .. 婖傖猁暮翹虜華硊
#endif
} STDiskFindControl;

struct STFileNodeInfo
{
	INT8S		disk;					//disk
	INT16U		attr;					//set the attribute to be found
	INT8S		extname[4];				//extension
	INT8S		*pExtName;				//for extend disk find funtion, support find multi extend name
	INT8S		*path;					//the file in the the path to be found
	INT16U		*FileBuffer;			//buffer point for file nodes
	INT16U		FileBufferSize;			//buffer size, every 20 words contain a file node, so must be multiple of 20
	INT16U		*FolderBuffer;			//buffer point for folder nodes
	INT16U		FolderBufferSize;		//buffer size, every 20 words contain a file node, so must be multiple of 20

	// the following parameter user do not care
	INT8S		flag;
	INT8U		root_dir_flag;			// if the root folder have found the file, this flag is setted
	// 08.02.27 add for search more then one kinds extern name of file
	INT16U		MarkDistance;
	INT32S		MaxFileNum;
	INT32S		MaxFolderNum;
	// 08.02.27 add end

	INT32S		(*filter)(INT8S *name);
};

typedef struct
{
	INT8U	name[11 + 1];
	INT16U	f_time;
	INT16U	f_date;
} STVolume;

// file system global variable
#if _OPERATING_SYSTEM != _OS_NONE
#if _OPERATING_SYSTEM == _OS_UCOS2
extern OS_EVENT *gFS_sem;
#elif _OPERATING_SYSTEM == _OS_FREERTOS
extern xSemaphoreHandle gFS_sem;
#endif
#endif

extern INT16U			gUnicodePage;
extern const struct nls_table	nls_ascii_table;
//extern const struct nls_table	nls_arabic_table;
extern const struct nls_table	nls_cp936_table;
extern const struct nls_table	nls_cp950_table;
extern const struct nls_table	nls_cp932_table;
//extern const struct nls_table	nls_cp1252_table;

/***************************************************************************/
/*        F U N C T I O N    D E C L A R A T I O N S	     			   */
/***************************************************************************/
//========================================================
//Function Name:	fs_get_version
//Syntax:		const char *fs_get_version(void);
//Purpose:		get file system library version
//Note:			the return version string like "GP$xyzz" means ver x.y.zz
//Parameters:   void
//Return:		the library version
//=======================================================
extern const char *fs_get_version(void);

//========================================================
//Function Name:	file_search_start
//Syntax:		INT32S file_search_start(struct STFileNodeInfo *stFNodeInfo, STDiskFindControl *pstDiskFindControl)
//Purpose:		search all the files of disk start
//Note:
//Parameters:   stFNodeInfo
//				pstDiskFindControl
//Return:		0 means SUCCESS, -1 means faile
//=======================================================
extern INT32S file_search_start(struct STFileNodeInfo *stFNodeInfo, STDiskFindControl *pstDiskFindControl);

//========================================================
//Function Name:	file_search_continue
//Syntax:		INT32S file_search_continue(struct STFileNodeInfo *stFNodeInfo, STDiskFindControl *pstDiskFindControl)
//Purpose:		search all the files of disk continue
//Note:
//Parameters:   stFNodeInfo
//				pstDiskFindControl
//Return:		0 means SUCCESS, 1 means search end, -1 means faile
//=======================================================
extern INT32S file_search_continue(struct STFileNodeInfo *stFNodeInfo, STDiskFindControl *pstDiskFindControl);

extern INT32S file_search_in_folder_start(struct STFileNodeInfo *stFNodeInfo, STDiskFindControl *pstDiskFindControl);
extern INT32S file_search_in_folder_continue(struct STFileNodeInfo *stFNodeInfo, STDiskFindControl *pstDiskFindControl);

//========================================================
//Function Name:	getfirstfile
//Syntax:		f_ppos getfirstfile(INT16S dsk, CHAR *extname, struct sfn_info* f_info, INT16S attr);
//Purpose:		find the first file of the disk(will find into the folder)
//Note:
//Parameters:   dsk, extname, f_info, attr
//Return:		f_ppos
//=======================================================
extern f_ppos getfirstfile(INT16S dsk, CHAR *extname, struct sfn_info* f_info, INT16S attr,INT32S (*filter)(INT8S* str));

//========================================================
//Function Name:	getnextfile
//Syntax:		f_ppos getnextfile(INT16S dsk, CHAR *extname, struct sfn_info* f_info, INT16S attr);
//Purpose:		find the next file of the disk(will find into the folder)
//Note:
//Parameters:   dsk, extname, f_info, attr
//Return:		f_ppos
//=======================================================
extern f_ppos getnextfile(INT16S dsk, CHAR * extname, struct sfn_info* f_info, INT16S attr,INT32S (*filter)(INT8S* str));

//Function Name:	getpaperfirstfile
//Syntax:		f_ppos getpaperfirstfile(CHAR *path, CHAR *extname, struct sfn_info* f_info, INT16S attr);

//Purpose:		find the next file of the Folder(will find into the folder)
//Note:
//Parameters:   dsk, extname, f_info, attr
//Return:		f_ppos
//=======================================================
extern f_ppos getpaperfirstfile(CHAR *path, CHAR *extname, struct sfn_info* f_info, INT16S attr, INT32S (*filter)(INT8S *str));

//========================================================
//Function Name:	getpapernextfile
//Syntax:		f_ppos getpapernextfile(CHAR *extname,struct sfn_info* f_info, INT16S attr);
//Purpose:		find the next file of the Folder(will find into the folder)
//Note:
//Parameters:   dsk, extname, f_info, attr
//Return:		f_ppos
//=======================================================
extern f_ppos getpapernextfile(CHAR *extname,struct sfn_info* f_info, INT16S attr, INT32S (*filter)(INT8S *str));

//========================================================
//Function Name:	sfn_open
//Syntax:		INT16S sfn_open(f_ppos ppos);
//Purpose:		open the file that getfirstfile/getnextfile find
//Note:
//Parameters:   ppos
//Return:		file handle
//=======================================================
extern INT16S sfn_open(f_ppos ppos);

//========================================================
//Function Name:	sfn_stat
//Syntax:		INT16S sfn_stat(INT16S fd, struct sfn_info *sfn_info);
//Purpose:		get file attribute of an opened file
//Note:
//Parameters:   fd, sfn_info
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S sfn_stat(INT16S fd, struct sfn_info *sfn_info);

//========================================================
//Function Name:	GetFileInfo
//Syntax:		INT16S FsgetFileInfo(INT16S fd, struct f_info *f_info);
//Purpose:		get long file name infomation through file handle
//Note:
//Parameters:   fd, f_info
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S FsgetFileInfo(INT16S fd, struct f_info *f_info);

//========================================================
//Function Name:	GetFileInfo
//Syntax:		INT16S GetFileInfo(f_ppos ppos, struct f_info *f_info);
//Purpose:		get long file name infomation that getfirstfile/getnextfile find
//Note:
//Parameters:   ppos, f_info
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S GetFileInfo(f_ppos ppos, struct f_info *f_info);

//========================================================
//Function Name:	GetFolderInfo
//Syntax:		INT16S GetFolderInfo(f_ppos ppos, struct f_info *f_info);
//Purpose:		get long folder name infomation that getfirstfile/getnextfile find
//Note:
//Parameters:   ppos, f_info
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S GetFolderInfo(f_ppos ppos, struct f_info *f_info);

//========================================================
//Function Name:	sfn_unlink
//Syntax:		INT16S sfn_unlink(f_ppos ppos);
//Purpose:		delete the file that getfirstfile/getnextfile find
//Note:
//Parameters:   ppos
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S sfn_unlink(f_ppos ppos);

//========================================================
//Function Name:	StatFileNumByExtName
//Syntax:		INT16S StatFileNumByExtName(INT16S dsk, CHAR *extname, INT32U *filenum);
//Purpose:		get the file number of the disk that have the same extend name
//Note:
//Parameters:   dsk, extname, filenum
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S StatFileNumByExtName(INT16S dsk, CHAR *extname, INT32U *filenum);

//========================================================
//Function Name:	GetFileNumEx
//Syntax:		INT16S GetFileNumEx(struct STFileNodeInfo *stFNodeInfo, INT32U *nFolderNum, INT32U *nFileNum);
//Purpose:		get the file number and the folder number of the disk that have the same extend name
//Note:
//Parameters:   stFNodeInfo, nFolderNum, nFileNum
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S GetFileNumEx(struct STFileNodeInfo *stFNodeInfo, INT32U *nFolderNum, INT32U *nFileNum);

//========================================================
//Function Name:	GetFileNodeInfo
//Syntax:		f_ppos GetFileNodeInfo(struct STFileNodeInfo *stFNodeInfo, INT32U nIndex, struct sfn_info* f_info);
//Purpose:		get the file node infomation
//Note:			before run this function, ensure you have execute the function "GetFileNumEx()"
//				0 <= nIndex < nMaxFileNum
//Parameters:   stFNodeInfo, nIndex, f_info
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern f_ppos GetFileNodeInfo(struct STFileNodeInfo *stFNodeInfo, INT32U nIndex, struct sfn_info* f_info);

//========================================================
//Function Name:	GetFolderNodeInfo
//Syntax:		f_ppos GetFolderNodeInfo(struct STFileNodeInfo *stFNodeInfo, INT32U nFolderIndex, struct sfn_info* f_info);
//Purpose:		get the folder node infomation
//Note:			before run this function, ensure you have execute the function "GetFileNumEx()"
//				0 <= nIndex < nMaxFileNum
//Parameters:   stFNodeInfo, nFolderIndex, f_info
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern f_ppos GetFolderNodeInfo(struct STFileNodeInfo *stFNodeInfo, INT32U nFolderIndex, struct sfn_info* f_info);

//========================================================
//Function Name:	GetFileNumOfFolder
//Syntax:		INT16S GetFileNumOfFolder(struct STFileNodeInfo *stFNodeInfo, INT32U nFolderIndex, INT16U *nFile);
//Purpose:		get the file number of a folder
//Note:			before run this function, ensure you have execute the function "GetFileNumEx()"
//Parameters:   stFNodeInfo, nFolderIndex, nFile
//Return:		0, SUCCESS
//				-1, FAILE
//================================== =====================
extern INT16S GetFileNumOfFolder(struct STFileNodeInfo *stFNodeInfo, INT32U nFolderIndex, INT16U *nFile);

//========================================================
//Function Name:	FolderIndexToFileIndex
//Syntax:		INT16S FolderIndexToFileIndex(struct STFileNodeInfo *stFNodeInfo, INT32U nFolderIndex, INT32U *nFileIndex);
//Purpose:		convert folder id to file id(the index number of first file in this folder)
//Note:			before run this function, ensure you have execute the function "GetFileNumEx()"
//Parameters:   stFNodeInfo, nFolderIndex, nFileIndex
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S FolderIndexToFileIndex(struct STFileNodeInfo *stFNodeInfo, INT32U nFolderIndex, INT32U *nFileIndex);

//========================================================
//Function Name:	FileIndexToFolderIndex
//Syntax:		INT16S FileIndexToFolderIndex(struct STFileNodeInfo *stFNodeInfo, INT32U nFileIndex, INT32U *nFolderIndex);
//Purpose:		convert file id to folder id(find what folder id that the file is in)
//				fileindex繭file痤folderindex
//Note:			before run this function, ensure you have execute the function "GetFileNumEx()"
//Parameters:   stFNodeInfo, nFolderIndex, nFileIndex
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S FileIndexToFolderIndex(struct STFileNodeInfo *stFNodeInfo, INT32U nFileIndex, INT32U *nFolderIndex);

//========================================================
//Function Name:	get_fnode_pos
//Syntax:		INT16S get_fnode_pos(f_pos *fpos);
//Purpose:		get the file node position after findfirst/findnext, and then you can open this file by sfn_open
//Note:			before run this function, ensure you have execute the function "_findfirst()/_findnext()"
//Parameters:   fpos
//Return:		0, SUCCESS
//=======================================================
extern INT16S get_fnode_pos(f_pos *fpos);

//f_ppos getfirstfileEx(INT8S *path, CHAR *extname, struct sfn_info *f_info, INT16S attr);
//f_ppos getnextfileEx(CHAR * extname, struct sfn_info* f_info, INT16S attr);

//========================================================
//Function Name:	dosdate_decode
//Syntax:		void dosdate_decode(INT16U dos_date, INT16U *pyear, INT8U *pmonth, INT8U *pday);
//Purpose:		convert the dos_data to year, month, day
//Note:
//Parameters:   dos_date, pyear, pmonth, pday
//Return:		void
//=======================================================
extern void dosdate_decode(INT16U dos_date, INT16U *pyear, INT8U *pmonth, INT8U *pday);

//========================================================
//Function Name:	dostime_decode
//Syntax:		void dostime_decode(INT16U dos_time, INT8U *phour, INT8U *pminute, INT8U *psecond);
//Purpose:		convert the dos_time to hour, minute, second
//Note:
//Parameters:   dos_time, phour, pminute, psecond
//Return:		void
//=======================================================
extern void dostime_decode(INT16U dos_time, INT8U *phour, INT8U *pminute, INT8U *psecond);

//========================================================
//Function Name:	time_decode
//Syntax:		INT8S *time_decode(INT16U *tp, CHAR *timec);
//Purpose:		convert *tp to a string like "hh:mm:ss"
//Note:
//Parameters:   tp, timec
//Return:		the point of string
//=======================================================
extern INT8S *time_decode(INT16U *tp, CHAR *timec);

//========================================================
//Function Name:	date_decode
//Syntax:		INT8S *date_decode(INT16U *dp, CHAR *datec);
//Purpose:		convert *dp to a string like "yyyy-mm-dd"
//Note:
//Parameters:   dp, datec
//Return:		the point of string
//=======================================================
extern INT8S *date_decode(INT16U *dp, CHAR *datec);

//========================================================
//Function Name:	fs_safexit
//Syntax:		void fs_safexit(void);
//Purpose:		close all the opened files except the registed file
//Note:
//Parameters:   NO
//Return:		void
//=======================================================
extern void fs_safexit(void);

//========================================================
//Function Name:	fs_registerfd
//Syntax:		void fs_registerfd(INT16S fd);
//Purpose:		regist opened file so when you call function fs_safexit() this file will not close
//Note:
//Parameters:   fd
//Return:		void
//=======================================================
extern void fs_registerfd(INT16S fd);

//========================================================
//Function Name:	disk_safe_exit
//Syntax:		void disk_safe_exit(INT16S dsk);
//Purpose:		close all the opened files of the disk
//Note:
//Parameters:   dsk
//Return:		void
//=======================================================
extern void disk_safe_exit(INT16S dsk);

//========================================================
//Function Name:	open
//Syntax:		INT16S open(CHAR *path, INT16S open_flag);
//Purpose:		open/creat file
//Note:
//Parameters:   path, open_flag
//Return:		file handle
//=======================================================
extern INT16S fs_open(CHAR *path, INT16S open_flag);

//========================================================
//Function Name:	close
//Syntax:		INT16S close(INT16S fd);
//Purpose:		close file
//Note:
//Parameters:   fd
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S fs_close(INT16S fd);

//========================================================
//Function Name:	read
//Syntax:		INT32S read(INT16S fd, INT32U buf, INT32U size);
//Purpose:		read data
//Note:			the buffer is BYTE address, the size is BYTE size
//Parameters:   fd, buf, size
//Return:		really read size
//=======================================================
extern INT32S fs_read(INT16S fd, INT32U buf, INT32U size);

//========================================================
//Function Name:	write
//Syntax:		INT32S write(INT16S fd, INT32U buf, INT32U size);
//Purpose:		write data
//Note:			the buffer is BYTE address, the size is BYTE size
//Parameters:   fd, buf, size
//Return:		really write size
//=======================================================
extern INT32S fs_write(INT16S fd, INT32U buf, INT32U size);

//========================================================
//Function Name:	lseek64
//Syntax:		INT64S lseek64(INT16S handle, INT64S offset0,INT16S fromwhere);
//Purpose:		change data point of file
//Note:			use lseek(fd, 0, SEEK_CUR) can get current offset of file.
//Parameters:   fd, offset, fromwhere
//Return:		data point
//=======================================================
extern INT64S lseek64(INT16S handle, INT64S offset0,INT16S fromwhere);

//========================================================
//Function Name:	lseek
//Syntax:		INT32S lseek(INT16S fd,INT32S offset,INT16S fromwhere);
//Purpose:		change data point of file
//Note:			use lseek(fd, 0, SEEK_CUR) can get current offset of file.
//Parameters:   fd, offset, fromwhere
//Return:		data point
//=======================================================
extern INT32S lseek(INT16S handle, INT32S offset0,INT16S fromwhere);

//========================================================
//Function Name:	unlink
//Syntax:		INT16S unlink(CHAR *filename);
//Purpose:		delete the file
//Note:
//Parameters:   filename
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S unlink(CHAR *filename);

//========================================================
//Function Name:	rename
//Syntax:		INT16S _rename(CHAR *oldname, CHAR *newname);
//Purpose:		change file name
//Note:
//Parameters:   oldname, newname
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _rename(CHAR *oldname, CHAR *newname);

//========================================================
//Function Name:	mkdir
//Syntax:		INT16S mkdir(CHAR *pathname);
//Purpose:		cread a folder
//Note:
//Parameters:   pathname
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S mkdir(CHAR *pathname);

//========================================================
//Function Name:	rmdir
//Syntax:		INT16S rmdir(CHAR *pathname);
//Purpose:		delete a folder
//Note:			the folder must be empty
//Parameters:   pathname
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S rmdir(CHAR *pathname);

//========================================================
//Function Name:	chdir
//Syntax:		INT16S chdir(CHAR *path);
//Purpose:		change current path to new path
//Note:
//Parameters:   path
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S chdir(CHAR *path);

//========================================================
//Function Name:	getcwd
//Syntax:		INT32U getcwd(CHAR *buffer, INT16S maxlen );
//Purpose:		get current path
//Note:
//Parameters:   buffer, maxlen
//Return:		the path name string point
//=======================================================
extern INT32U getcwd(CHAR *buffer, INT16S maxlen );

//========================================================
//Function Name:	fstat
//Syntax:		INT16S fstat(INT16S handle, struct stat_t *statbuf);
//Purpose:		get file infomation
//Note:			the file must be open
//Parameters:   handle, statbuf
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S fstat(INT16S handle, struct stat_t *statbuf);

//========================================================
//Function Name:	stat
//Syntax:		INT16S stat(CHAR *path, struct stat_t *statbuf);
//Purpose:		get file infomation
//Note:
//Parameters:   path, statbuf
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S stat(CHAR *path, struct stat_t *statbuf);

//========================================================
//Function Name:	_findfirst
//Syntax:		INT16S _findfirst(CHAR *name, struct f_info *f_info, INT16U attr);
//Purpose:		find the first file in one folder
//Note:
//Parameters:   name, f_info, attr
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _findfirst(CHAR *name, struct f_info *f_info, INT16U attr);

//========================================================
//Function Name:	_findnext
//Syntax:		INT16S _findnext(struct f_info *f_info);
//Purpose:		find next file in one folder
//Note:
//Parameters:   f_info
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _findnext(struct f_info *f_info);

//========================================================
//Function Name:	_getdiskfree
//Syntax:		INT16S _getdiskfree(INT16S dsk, struct _diskfree_t *st_free);
//Purpose:		get disk total space and free space
//Note:
//Parameters:   dsk, st_free
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _getdiskfree(INT16S dsk, struct _diskfree_t *st_free);

//========================================================
//Function Name:	vfsFreeSpace
//Syntax:		INT32S vfsFreeSpace(INT16S driver);
//Purpose:		get disk free space
//Note:
//Parameters:   dsk, st_free
//Return:		the free space of the disk
//=======================================================
//INT32S vfsFreeSpace(INT16S driver);
extern INT64U vfsFreeSpace(INT16S driver);

//========================================================
//Function Name:	_changedisk
//Syntax:		INT16S _changedisk(INT8U disk);
//Purpose:		change current disk to another disk
//Note:
//Parameters:   disk
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
#define _changedisk     fs_changedisk
extern INT16S _changedisk(INT8U disk);

//========================================================
//Function Name:	_copy
//Syntax:		INT16S _copy(CHAR *path1, CHAR *path2);
//Purpose:		copy file
//Note:
//Parameters:   srcfile, destfile
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
INT16S _copy(CHAR *path1, CHAR *path2);

//========================================================
//Function Name:	fs_init
//Syntax:		void fs_init(void);
//Purpose:		initial all file system global variable
//Note:
//Parameters:   NO
//Return:		void
//=======================================================
extern void fs_init(void);

//========================================================
//Function Name:	fs_uninit
//Syntax:		INT16S fs_uninit(void);
//Purpose:		free file system resource, and unmount all disk
//Note:
//Parameters:   NO
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S fs_uninit(void);

//========================================================
//Function Name:	tellcurrentfiledir
//Syntax:		INT16U tellcurrentfiledir(void);
//Purpose:		get current directory entry point
//Note:
//Parameters:   NO
//Return:		directory entry point
//=======================================================
extern INT16U tellcurrentfiledir(void);

//========================================================
//Function Name:	telldir
//Syntax:		INT16U telldir(void);
//Purpose:		get next directory entry point
//Note:
//Parameters:   NO
//Return:		directory entry point
//=======================================================
extern INT16U telldir(void);

//========================================================
//Function Name:	seekdir
//Syntax:		void seekdir(INT16U pos);
//Purpose:		set directory entry point, and next time, if you call _findnext(),
//				the function will find file from this point
//Note:
//Parameters:   directory entry point
//Return:		NO
//=======================================================
extern void seekdir(INT16U pos);     //the parameter "pos" must be the return value of "telldir"

//========================================================
//Function Name:	rewinddir
//Syntax:		void rewinddir(void);
//Purpose:		reset directory entry point to 0
//Note:
//Parameters:   NO
//Return:		NO
//=======================================================
extern void rewinddir(void);

//========================================================
//Function Name:	_setfattr
//Syntax:		INT16S _setfattr(CHAR *filename, INT16U attr);
//Purpose:		set file attribute
//Note:
//Parameters:   filename, attr
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _setfattr(CHAR *filename, INT16U attr);

//========================================================
//Function Name:	_setdirattr
//Syntax:		INT16S _setdirattr(CHAR *dirname, INT16U attr);
//Purpose:		set dir attribute
//Note:
//Parameters:   filename, attr
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _setdirattr(CHAR *dirname, INT16U attr);

//========================================================
//Function Name:	_getdirattr
//Syntax:		INT16S _getdirattr(CHAR *dirname, INT16U *attr);
//Purpose:		get dir attribute
//Note:
//Parameters:   filename, attr
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _getdirattr(CHAR *dirname, INT16U *attr);

//========================================================
//Function Name:	_devicemount
//Syntax:		INT16S _devicemount(INT16S disked);
//Purpose:		mount disk, then you can use the disk
//Note:
//Parameters:   disk
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _devicemount(INT16S disked);

//========================================================
//Function Name:	_deviceunmount
//Syntax:		INT16S _deviceunmount(INT16S disked);
//Purpose:		unmount disk
//Note:
//Parameters:   disk
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _deviceunmount(INT16S disked);

//========================================================
//Function Name:	_getfserrcode
//Syntax:		INT16S _getfserrcode(void);
//Purpose:		get error code(see error.h)
//Note:
//Parameters:   NO
//Return:		error code
//=======================================================
extern INT16S _getfserrcode(void);

//========================================================
//Function Name:	_clsfserrcode
//Syntax:		void _clsfserrcode(void);
//Purpose:		clear error code to 0
//Note:
//Parameters:   NO
//Return:		void
//=======================================================
extern void _clsfserrcode(void);

//========================================================
//Function Name:	_format
//Syntax:		INT16S _format(INT8U drv, INT8U fstype);
//Purpose:		format disk to FAT32 or FAT16 type
//Note:
//Parameters:   dsk, fstype
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _format(INT8U drv, INT8U fstype);

//========================================================
//Function Name:	_deleteall
//Syntax:		INT16S _deleteall(CHAR *filename);
//Purpose:		delete all file and folder in one folder
//Note:
//Parameters:   path
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S _deleteall(CHAR *filename);

//========================================================
//Function Name:	GetSectorsPerCluster
//Syntax:		INT16U GetSectorsPerCluster(INT16U dsk)
//Purpose:		get Sector number per cluster
//Note:
//Parameters:   dsk
//Return:		sector number
//=======================================================
extern INT16U GetSectorsPerCluster(INT16U dsk);

//========================================================
//Function Name:	_GetCluster
//Syntax:		INT32S _GetCluster(INT16S fd);
//Purpose:		get cluster id that data point now locate
//Note:
//Parameters:   fd
//Return:		cluster id
//=======================================================
extern INT32S _GetCluster(INT16S fd);

//========================================================
//Function Name:	Clus2Phy
//Syntax:		INT32S Clus2Phy(INT16U dsk, INT32U cl_no);
//Purpose:		convert cluster id to sector address
//Note:
//Parameters:   dsk, cl_no
//Return:		sector address
//=======================================================
extern INT32S Clus2Phy(INT16U dsk, INT32U cl_no);

//========================================================
//Function Name:	DeletePartFile
//Syntax:		INT16S DeletePartFile(INT16S fd, INT32U offset, INT32U length);
//Purpose:		delete part of file, from "offset", delete "length" byte
//Note:			the file system will convert the "offset" and "length" to cluster size
//Parameters:   fd, offset, length
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S DeletePartFile(INT16S fd, INT32U offset, INT32U length);

//========================================================
//Function Name:	InserPartFile
//Syntax:		INT16S InserPartFile(INT16S tagfd, INT16S srcfd, INT32U tagoff, INT32U srclen);
//Purpose:		insert the src file to tag file
//Note:			the file system will convert the "offset" and "length" to cluster size
//Parameters:   tagfd, srcfd, tagoff, srclen
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S InserPartFile(INT16S tagfd, INT16S srcfd, INT32U tagoff, INT32U srclen);

//========================================================
//Function Name:	InserPartFile
//Syntax:		INT16 InserPartFile(INT16 tagfd, INT16 srcfd, INT32U tagoff, INT32U srclen)
//Purpose:		split tag file into two file, one is remain in tag file, one is in src file
//Note:			the file system will convert the "offset" and "length" to cluster size
//Parameters:   tagfd, srcfd, splitpoint
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S SplitFile(INT16S tagfd, INT16S srcfd, INT32U splitpoint);

//========================================================
//Function Name:	ChangeCodePage
//Syntax:		INT16U ChangeCodePage(INT16U wCodePage);
//Purpose:		select unicode page
//Note:			if the code page is not exsit, the file system will default change code page to "ascii"
//Parameters:   wCodePage
//Return:		code page
//=======================================================
extern INT16U ChangeCodePage(INT16U wCodePage);

//========================================================
//Function Name:	GetCodePage
//Syntax:		INT16U GetCodePage(void);
//Purpose:		get unicode page
//Note:
//Parameters:   NO
//Return:		code page
//=======================================================
extern INT16U GetCodePage(void);

//========================================================
//Function Name:	ChangeUnitab
//Syntax:		INT16S ChangeUnitab(struct nls_table *st_nls_table);
//Purpose:		change unicode convert struct
//Note:
//Parameters:   st_nls_table
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S ChangeUnitab(struct nls_table *st_nls_table);

//========================================================
//Function Name:	checkfattype
//Syntax:		INT16S checkfattype(INT8U disk);
//Purpose:		get the fat type of the disk(FAT16 or FAT32)
//Note:
//Parameters:   disk
//Return:		fat type
//=======================================================
extern INT16S checkfattype(INT8U disk);

//========================================================
//Function Name:	UpdataDir
//Syntax:		INT16S UpdataDir(INT16S fd);
//Purpose:		updata dir information but not close the file
//Note:
//Parameters:   fd
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S UpdataDir(INT16S fd);

//========================================================
//Function Name:	FileRepair
//Syntax:		INT16S FileRepair(INT16S fd);
//Purpose:		if the file is destroy for some reason, this function will repair the file
//Note:			it can't deal with some complicated condition
//Parameters:   fd
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S FileRepair(INT16S fd);

//========================================================
//Function Name:	sformat
//Syntax:		INT16S sformat(INT8U drv, INT32U totalsectors, INT32U realsectors);
//Purpose:		format some disk that size is less than 16 MB
//Note:
//Parameters:   drv, totalsectors, realsectors
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S sformat(INT8U drv, INT32U totalsectors, INT32U realsectors);

//========================================================
//Function Name:	GetDiskOfFile
//Syntax:		INT16S GetDiskOfFile(INT16S fd);
//Purpose:		get the disk id of an opened file
//Note:
//Parameters:   fd
//Return:		disk id, 0 is disk "A", 1 is disk "B", and itc...
//=======================================================
extern INT16S GetDiskOfFile(INT16S fd);

//========================================================
//Function Name:	CreatFileBySize
//Syntax:		INT16S CreatFileBySize(CHAR *path, INT32U size);
//Purpose:		creat a file, and allocate "size" byte space
//Note:			size is byte size
//Parameters:   filename, size
//Return:		file handle
//=======================================================
extern INT16S CreatFileBySize(CHAR *path, INT32U size);

//========================================================
//Function Name:	get_first_file_in_folder
//Syntax:		f_ppos get_first_file_in_folder(STDiskFindControl *pstDiskFindControl, INT8S *path, INT8S *extname, struct sfn_info* f_info, INT16S attr, INT8S find_child_dir);
//Purpose:		find the first file in a folder
//Note:
//Parameters:   pstDiskFindControl:
//				path: the folder to be found
//				extname: all the file have this extend name can be found. if "*", find all file
//				f_info: the file be found
//				find_child_dir: 1 means find the file in child folder, 0 means not find
//Return:		file position
//=======================================================
extern f_ppos get_first_file_in_folder(STDiskFindControl *pstDiskFindControl, INT8S *path, INT8S *extname, struct sfn_info* f_info, INT16S attr, INT8S find_child_dir, INT32S (*filter)(INT8S *str));

//========================================================
//Function Name:	get_next_file_in_folder
//Syntax:		f_ppos get_next_file_in_folder(STDiskFindControl *pstDiskFindControl, INT8S *path, INT8S *extname, struct sfn_info* f_info, INT16S attr, INT8S find_child_dir);
//Purpose:		find the next file in a folder
//Note:
//Parameters:   pstDiskFindControl:
//				path: the folder to be found
//				extname: all the file have this extend name can be found. if "*", find all file
//				f_info: the file be found
//				find_child_dir: 1 means find the file in child folder, 0 means not find
//Return:		file position
//=======================================================
extern f_ppos get_next_file_in_folder(STDiskFindControl *pstDiskFindControl, INT8S *path, INT8S *extname, struct sfn_info* f_info, INT16S attr, INT8S find_child_dir, INT32S (*filter)(INT8S *str));

//========================================================
//Function Name:	flie_cat
//Syntax:		flie_cat(INT16S file1_handle, INT16S file2_handle);
//Purpose:		cat two file to one file, file1_handle.
//Note:
//Parameters:   file1_handle, file2_handle
//Return:		0: success, other fail
//=======================================================
extern INT16S file_cat(INT16S file1_handle, INT16S file2_handle);

//========================================================
//Function Name:	unlink2
//Syntax:		INT16S unlink2(CHAR *filename);
//Purpose:		delete the file
//Note:
//Parameters:   filename
//Return:		0, SUCCESS
//				-1, FAILE
//=======================================================
extern INT16S unlink2(CHAR *filename);
//========================================================
//Function Name:	set_volume
//Syntax:		INT16S set_volume(INT8U disk_id, INT8U *p_volum);
//Purpose:		set dis volume name
//Note:
//Parameters:
//Return:		0: succeed   other :fail
//=======================================================
extern void set_exfatlink(INT8U disk_id,INT8U flags);//Exfat only


//========================================================
//Function Name:	get_freeclu_misc
//Syntax:		INT32U get_freeclu_misc(INT8U disk);
//Purpose:		get free cluster at info sector
//Note:
//Parameters:   disk unit
//Return:		free cluster
//=======================================================
extern INT32U get_freeclu_misc(INT8U disk);

//========================================================
//Function Name:	get_freearea_misc
//Syntax:		INT32U get_freearea_misc(INT8U disk);
//Purpose:		get free area at info sector
//Note:
//Parameters:   disk unit
//Return:		free area
//=======================================================
extern INT32U get_freearea_misc(INT8U disk);

//========================================================
//Function Name:	get_fatsize
//Syntax:		INT32U get_fatsize(INT8U disk);
//Purpose:		get fat size (bytes)
//Note:
//Parameters:   disk unit
//Return:		fat size(bytes)
//=======================================================
extern INT32U get_fatsize_misc(INT8U disk);

//========================================================
//Function Name:	read_fat_frist
//Syntax:		INT32U read_fat_frist(INT32U disk, INT32U buf, INT32U size);
//Purpose:		from the beginning to read fat
//Note:
//Parameters:   disk unit
//Return:		fat size(bytes)
//=======================================================
extern INT32U read_fat_frist_misc(INT32U disk, INT32U buf, INT32U size);


//========================================================
//Function Name:	read_fat_next
//Syntax:		INT32U read_fat_frist(INT32U disk, INT32U buf, INT32U size);
//Purpose:		Continue to read fat
//Note:
//Parameters:   disk unit
//Return:		fat size(bytes)
//=======================================================
extern INT32U read_fat_next_misc(INT32U disk, INT32U buf, INT32U size);

extern INT16S get_volume(INT8U disk_id, STVolume *pstVolume);
extern INT16S set_volume(INT8U disk_id, INT8U *p_volum);
extern INT16S handle_error_code (INT16S errcode);
extern INT16S gFS_errno;

// background unlink
extern INT32U FsBgUnlink_get_workmem_size(void);
extern INT32S FsBgUnlink_open(void* pWorkMem, INT32U prio);
extern INT32S FsBgUnlink_file(void* pWorkMem, CHAR *FileName, INT32U bgWorkFlag, INT32U msec);
extern INT32S FsBgUnlink_close(void* pWorkMem);

extern INT32U fs_sd_ms_plug_out_flag_get(void);
extern void fs_sd_ms_plug_out_flag_reset(void);
extern INT8U fs_usbh_plug_out_flag_get(void);
extern void fs_usbh_plug_out_flag_reset(void);


//#include "gplib_print_string.h"
//#include "gplib_jpeg.h"
//#include "gplib_jpeg_decode.h"
//#include "gplib_jpeg_encode.h"
//#include "gplib_mjpeg_decode.h"
//#include "gplib_progressive_jpeg_decode.h"

#endif 		// __GPLIB_H__
