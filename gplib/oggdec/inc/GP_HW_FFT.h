#define HW_FFT  1


//========================================================
// Function Name : GP_HW_FFT_RUN
// Syntax : void GP_HW_FFT_RUN(long *inbuf, long *outbuf)
// Purpose : main process
// Parameters : int *buf : InBuf addr
//              unsigned int nfft: fft point
// Return : none
//========================================================
extern void GP_HW_FFT_RUN(int *buf, int nfft);
