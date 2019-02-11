#ifndef AUDIODEV_H_
#define AUDIODEV_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief Defines the sampling format of the output device.
 *
 *  Currently supported audio device formats:
 *  AUDIODEV_SAMPLEFORMAT_MONO_16: Mono 16 bit.
 */
#define AUDIODEV_SAMPLEFORMAT_MONO_16  0

/**
 *  \brief When implemented, initialises the audio device interface.
 *
 *  \remarks The application must call this interface before any text
 *  to speech conversion functionality is used. However, if the application is
 *  using the library in streaming mode (filling the audio buffer in a loop), then
 *  there is no need to call this function. Additionally, this function can consist of
 *  an empty implementation in that case.
 */
extern void AUDIODEV_Init(void);

/**
 *  \brief When implemented, will instruct the audio device to process its internal next step.
 *  The application must call this function from within its main loop. The function may be implemented empty
 *  if the audio device of target hosting system does not require monitoring, or if the library is being used
 *  in streaming mode (filling the audio buffer in a loop).
 */
extern void AUDIODEV_Process(void);

/**
 *  \brief When implemented, opens a stream for audio playback.
 *  Unless used in streaming mode (filling the audio buffer in a loop) the
 *  library will call this function before attempting to write audio samples in order to produce audio output.
 *  If the library is being used in streaming mode, this function can have an empty implementation.
 */
extern void AUDIODEV_OpenPlayback(void);

/**
 *  \brief When implemented, closes the audio playback stream.
 *  Unless used in streaming mode (filling the audio buffer in a loop) the
 *  library will call this function when it no longer needs to write audio samples to the audio output.
 *  If the library is being used in streaming mode, this function can have an empty implementation.
 */
extern void AUDIODEV_ClosePlayback(void);

/**
 *  \brief When implemented, sets the volume of the audio device. The library will call this function,
 *  if the corresponding functional tag is encountered in the input text. If it is not necessary to enable
 *  the library to perform such action, the function may be implemented empty.
 *
 *  \param volume The volume in percentage (0% to 100%) of the audio output.
 */
extern void AUDIODEV_SetVolume(uint32_t volume);

/**
 *  \brief When implemented, sets the sampling rate and PCM format of the audio device.
 *  This function will be called by the library upon initialisation to set the proper audio
 *  output configuration. If desired, the function can also be implemented empty, provided that
 *  the audio device is configured by the application in the same mode as the library.
 *
 *  \param sampleRate The data rate, in Hz, at which the audio samples should be played back.
 *  \param format The PCM format in which the samples are provided. Currently, the only supported format
 *  is AUDIODEV_SAMPLEFORMAT_MONO_16: Which means Mono output with 16 bit samples.
 */
extern void AUDIODEV_SetDataRate(uint32_t sampleRate, uint32_t format);

/**
 *  \brief When implemented, writes audio samples to the audio output buffer.
 *  The format in which the samples are provided is previously set by
 *  a call to AUDIODEV_SetDataRate before calling this function. If the library is being used
 *  in streaming mode (filling the audio buffer in a loop). This function can be implemented empty
 *  and return nrOfSamples.
 *
 *  \param pData The pointer to audio sample data buffer.
 *  \param nrOfSamples The size of the audio sample data buffer in number of samples.
 *
 *  \return The actual number of samples written in the audio output buffer.
 *
 *  \see AUDIODEV_SetDataRate.
 */
extern uint32_t AUDIODEV_WriteSamples(const void * pData, uint32_t nrOfSamples);

/**
 *  \brief  When implemented, returns the number of samples still in the audio output buffer.
 *  The number of samples still in the buffer is returned. Considering DMA usage and / or other
 *  mechanisms present on the host platform, it is possible that some samples are still in the
 *  buffer, but have already been outputted. That does not constitute an anomaly in terms of
 *  system functionality. This situation is considered normal.
 *  If the library is being used in streaming mode (filling the audio buffer in a loop), this
 *  function may be implemented empty and return 0.
 *
 *  \return The number of samples still in the buffer.
 */
extern uint32_t AUDIODEV_GetNrOfQueuedSamples(void);

/**
 *  \brief When implemented, returns the amount of free space, in samples, of the audio output buffer.
 *  If the library is being used in streaming mode (filling the audio buffer in a loop), this
 *  function may be implemented empty and return 0.
 *
 *  \return Number of samples that can still be written into the audio output buffer.
 */
extern uint32_t AUDIODEV_GetFreeSampleQueueSpace(void);

/**
 *  \brief When implemented, returns the total size, in samples, of the audio output buffer.
 *  If the library is being used in streaming mode (filling the audio buffer in a loop), this
 *  function may be implemented empty and return 0.
 *
 *  \return The size of the audio output buffer, in samples.
 */
extern uint32_t AUDIODEV_GetSizeOfSampleQueue(void);

/**
 *  \brief When implemented, returns the number of samples played since the last call to AUDIODEV_OpenPlayback.
 *  If the library is being used in streaming mode (filling the audio buffer in a loop), this
 *  function may be implemented empty and return 0.
 *
 *  \return The number of played samples.
 *
 *  \see AUDIODEV_OpenPlayback.
 */
extern uint32_t AUDIODEV_GetPlayedSamples(void);

/**
 *  \brief When implemented, pauses the audio output.
 *  If the library is being used in streaming mode (filling the audio buffer in a loop), this
 *  function may be implemented empty.
 */
extern void AUDIODEV_Pause(void);

/**
 *  \brief When implemented, resumes the audio output.
 *  If the library is being used in streaming mode (filling the audio buffer in a loop), this
 *  function may be implemented empty.
 */
extern void AUDIODEV_Resume(void);

#ifdef __cplusplus
}
#endif

#endif  // AUDIODEV_H_
