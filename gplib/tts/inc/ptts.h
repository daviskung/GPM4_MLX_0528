#ifndef PTTS_H_
#define PTTS_H_

#include <stdint.h>




#ifdef PTTS_EXPORTS

// The following ifdef block is the standard way of creating macros which
// make exporting from a DLL simpler. All files within this DLL are compiled
// with the PTTS_EXPORTS symbol defined on the command line. this symbol
// should not be defined on any project that uses this DLL. This way any
// other project whose source files include this file see TTS_API
// functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#define PTTS_LIB
#define TTS_API __declspec(dllexport)
#elif defined(PTTS_IMPORTS)
#define TTS_API __declspec(dllimport)
#else
/**
 *  \brief Defines the TTS_API export macro.
 */
#define TTS_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief Defines all possible text (input & output) formats.
 */
typedef enum P_TTS_FORMAT
{
    P_TTS_PLAIN_TEXT = 0,  //!< Plain text format.
    P_TTS_MBROLA_FORMAT = 1,  //!< MBROLA format (phoneme level).
    P_TTS_SSML_FORMAT = 2,  //!< SSML format.
    P_TTS_SSML_FORMAT_DEB = 3,  //!< SSML format (store output in ssml.log).
    P_TTS_UNICODE_TEXT = 4  //!< Unicode text format.
} P_TTS_FORMAT;


/**
 *  \brief Frees the TTS interface and should be called if the text pronunciation can be ended.
 */
TTS_API void p_tts_ExitTTS(void);


/**
 *  \brief Changes current volume(in percentage) with low-/high-pass
 *  scaling and low-/high-pass frequency range.
 *  A positive scale is high-pass, a negative scale is low-pass.
 *  A scale of 0 is all-pass.
 *  A 100% range uses the full bandwidth
 *
 *  \param volume The change to the current volume in percentage. The value ranges between 0% and 400%.
 *  \param scale The high/low pass scaling value. The value ranges between -100% and 100%.
 *  \param range The frequency range to use. The value ranges is between 0% and 100%.
 */
TTS_API void p_tts_SetFilter(short volume, short scale, short range);

/**
 *  \brief Changes current speed to the provided value(in percentage).
 *  The speed parameter controls the speed at which the text is spoken, also called the speaking rate.
 *  The speaking rate can be increased or decreased.
 *  The parameter is a percentage of the normal speaking rate stored in the voice database.
 *  For example if the value is set to 80, then the speaking rate is decreased by 20%. The value is a
 *  16 bit number and will be limited between 50 (i.e. 50%) and 300 (i.e. 300%).
 *
 *  \param speed The change to apply to the current speed. In percentage.
 */
TTS_API void p_tts_SetSpeed(short speed);


/**
 *  \brief Enumerates all the possible event types for \ref Event.
 */
typedef enum P_TTS_EVENT
{
    P_TTS_MARKER_EVENT = 1,  //!< Marker event.
    P_TTS_TOKEN_EVENT = 2,  //!< Token event.
    P_TTS_SENTENCE_EVENT = 8,  //!< Sentence event.
    P_TTS_UNDEFINED_EVENT = 0x10000000  //!< Undefined event.
} P_TTS_EVENT;

/**
 *  \brief Contains data associated with the TTS library events such as the pronunciation of a new token.
 */
struct Event
{
    enum P_TTS_EVENT type;  //!< Event type.
    const char *text;  //!< Event text.
    unsigned textLENGTH;  //!< Event text length.
    unsigned inputINDEX;  //!< Index within the input text.
    unsigned inputLENGTH;  //!< Length within the input text.
    unsigned phoneINDEX;  //!< Phoneme index.
    unsigned phonePOS;  //!< Position within phone.
    unsigned long sampleINDEX;  //!< Index within the output samples.
};

/**
 *  \brief Retrieves the string corresponding to the emotion representation with a given index.
 *
 *  \param num The index of the emotion to retrieve.
 *
 *  \return The string with the emotion representation, or NULL if num is larger than total number of emotions.
 */
TTS_API const char* p_tts_EnumerateEmotions(int num);

/**
 *  \brief Retrieves the string corresponding to the predefined character representation with a given index.
 *
 *  \param num The index of the character to retrieve.
 *
 *  \return The string with the character representation, or NULL if num is larger than total number of characters.
 */
TTS_API const char* p_tts_EnumerateCharacters(int num);

/**
 *  \brief Initializes the TTS interface. Uses \p languageID to initialize the language and \p voiceID to initialize the voice.
 *  Both parameters can be NULL. NULL means 'use the default'.
 *  Before any text can be pronounced, this function must be called first.
 *
 *  EXAMPLES:
 *
 *  - To say "hello":
 *
 *      p_tts_InitTTS(NULL, NULL);
 *      p_tts_StartTTS("hello", 0, P_TTS_PLAIN_TEXT);
 *      while (!p_tts_IsFinished()) {};
 *
 *      p_tts_ExitTTS();
 *
 *  - To collect the synthesized samples in a buffer:
 *
 *      static short samples[32000];
 *      unsigned long numSamples;
 *
 *      p_tts_InitTTS(NULL, NULL);
 *      p_tts_CollectSamples(samples, 32000);
 *
 *      p_tts_StartTTS("hello", 0, P_TTS_PLAIN_TEXT);
 *      numSamples = p_tts_GetNumSamples(samples);
 *
 *      p_tts_ExitTTS();
 *
 *         :
 *         :  process the samples
 *         :
 *
 *  - To use TTS in a streaming application:
 *
 *      static short samples[10000];
 *      static struct Event events[1000];
 *      unsigned long numSamples;
 *      unsigned numEvents;
 *
 *      p_tts_InitTTS(NULL, NULL);
 *      p_tts_CollectSamples(samples, 10000);
 *      p_tts_CollectEvents(events, 1000, P_TTS_TOKEN_EVENT);
 *      p_tts_StartTTS("hello", 0, P_TTS_PLAIN_TEXT);
 *
 *      while ((numSamples = p_tts_GetNumSamples(&numEvents)) > 0 ||
 *             (numEvents > 0) ||
 *             (!p_tts_IsFinished()))
 *       {
 *         if (numSamples > 0 || numEvents > 0)
 *         {
 *            :
 *            :  process the data
 *            :
 *
 *            p_tts_FlushSamples();
 *         }
 *      }
 *
 *      p_tts_ExitTTS();
 *
 *  \param languageID Pointer to the language string representation.
 *  \param voiceID Pointer to the voice string representation.
 *
 *  \return On success returns 0. On error returns -1.
 */
TTS_API int p_tts_InitTTS(const char languageID[], const char voiceID[]);

/**
 *  \brief Instructs the library to collect events in the internal event buffer filtered according to an event mask.
 *  If this buffer gets full before the synthesis ends, the remaining events are discarded. As a rule of thumb
 *  1/10 of the size of the sample buffer should be used. This function must be called before \ref p_tts_StartTTS is
 *  called.
 *
 *  \param events The pointer to the buffer of events.
 *  \param maxNumEvents The size of the events buffer.specifies collection buffer size. Minimum is 1.
 *  \param eventMask The filter indicating which events to collect. The filter mask is made of on or more flags
 *  of the \ref P_TTS_EVENT enumeration. In order to use multiple flags a binary or (|) operation can be used.
 *
 *  \return On success returns 0. On error returns -1.
 */
TTS_API int p_tts_CollectEvents(
                struct Event* events,
                unsigned maxNumEvents,
                unsigned eventMask);


/**
 *  \brief Instructs the library to collect the synthesized audio samples in a buffer to operate in streaming mode.
 *  The synthesized samples are stored in this buffer instead of being played through the audio device.
 *
 *  A small buffer-size will slow the synthesis down. A buffer size of at least 250ms is recommended.
 *
 *  This function should be called before \ref p_tts_GetNumSamples and can be called multiple times to support multiple
 *  audio buffers.
 *
 *  The synthesis will be blocked if the buffer gets full and will continue as soon
 *  as the buffer is freed by \ref p_tts_FlushSamples.
 *
 *  Use \ref p_tts_GetNumSamples to get the number of the collected samples and \ref p_tts_FlushSamples
 *  to flush the old data.
 *
 *  \param buffer Pointer to the buffer where samples will be stored.
 *  \param bufferLEN The length of buffer.
 *
 *  \return On success returns 0. On error returns -1.
 */
TTS_API int p_tts_CollectSamples(short* buffer, unsigned long bufferLEN);

/**
 *  \brief Enable token collection (in an internal buffer).
 *  This function should be called before \ref p_tts_StartTTS.
 *
 *  Use \ref p_tts_GetToken to get the collected data.
 */
TTS_API void p_tts_CollectTokens(void);


/**
 *  \brief Flushes the samples that were returned by the last call to
 *  p_tts_GetNumSamples(). Flushing frees memory for new audio samples.
 */
TTS_API void p_tts_FlushSamples(void);

/**
 *  \brief Gets the current emotion.
 *
 *  \return Pointer to emotion name. See \ref p_tts_SetEmotion for all possible emotions.
 */
TTS_API const char* p_tts_GetEmotion(void);

/**
 *  \brief Gets the current head size.
 *
 *  \return The head size settings in percentage or an -1 on error.
 */
TTS_API int p_tts_GetHeadSize(void);

/**
 *  \brief Gets the current language info.
 *
 *  \returns The pointer to language info string.
 */
TTS_API const char* p_tts_GetLanguageInfo(void);


/**
 *  \brief Returns the number of synthesized samples in the external sample buffer.
 *  If \p numEvents is not NULL, this parameter is set to the number of generated
 *  events in the external event buffer. It is necessary to call \ref p_tts_CollectSamples
 *  before calling this function in order to provide the library with a buffer to store the samples.
 *  When the function returns the buffer provided to \ref p_tts_CollectSamples will be full with
 *  the audio samples.
 *
 *  This functions blocks until the sample buffer provided to \ref p_tts_CollectSamples is full or the
 *  end of the speech generation has been reached. If needed, this function can be used in combination
 *  with \ref p_tts_DoNextStep() in a loop to make it non-blocking. The mechanism consists in calling
 *  \ref p_tts_DoNextStep() in a loop until it returns either 0 or -1. At this point, calling this function
 *  will return immediately with the number of samples contained in the buffer.
 *
 *  \param numEvents Output parameter that will contain the number of events during synthesis.
 *
 *  \return The number of synthesised samples.
 */
TTS_API unsigned long p_tts_GetNumSamples(unsigned* numEvents);

/**
 *  \brief Gets the current pitch value.
 *
 *  \return The pitch value in Hz or -1 on error.
 */
TTS_API int p_tts_GetPitch(void);

/**
 *  \brief Gets the sample rate in Hz.
 *
 *  \return The sample rate in Hz.
 */
TTS_API unsigned long p_tts_GetSampleRate(void);


/**
 *  \brief Gets the voice information as string.
 *
 *  \return The pointer to the current voice information string.
 */
TTS_API const char* p_tts_GetVoiceInfo(void);

/**
 *  \brief Gets the current speed setting in percentage.
 *  See \ref p_tts_SetSpeed.
 *
 *  \return The current speed in percentage or -1 if an error occurred.
 */
TTS_API int p_tts_GetSpeed(void);

/**
 *  \brief Gets a token event from internal buffer.
 *  A token is the basic unit that the TTS system uses to group the
 *  input characters into words, numbers, abbreviations etc.
 *  If the return value is -1, then there are no tokens available, which means the
 *  next sentence is being synthesised and the offset position should be reset.
 *
 *  \param event The pointer to event to be filled.
 *  \param offsetMSEC The time offset in milliseconds from the current play
 *  position.
 *
 *  \return If event is successfully filled, 0 is returned. Returns -1 if no token available.
 */
TTS_API int p_tts_GetToken(struct Event* event, int offsetMSEC);



/**
 *  \brief Returns the name of the current character.
 *
 *  \return pointer to character name string.
 */
TTS_API const char* p_tts_GetCharacter(void);

/**
 *  \brief Indicates whether processing is finished or not.
 *
 *  \return If processing is finished, returns 1, otherwise returns 0.
 */
TTS_API int p_tts_IsFinished(void);

/**
 *  \brief Sets the current emotion current emotion.
 *
 *  \param emotion The name of the new emotion.
 *  One of: "Neutral", "Friendly", "Angry",
 *  "Furious", "Drill", "Scared", "Emotional", "Weepy", "Excited",
 *  "Surprised", "Sad", "Disgusted" and "Whisper".
 *
 *  \return On success returns 0. On error returns -1.
 */
TTS_API int p_tts_SetEmotion(const char emotion[]);

/**
 *  \brief Sets the current head-size to \p h (in percentage).
 *
 *  \param headSize The value of the head size, in percentage.
 */
TTS_API void p_tts_SetHeadSize(int headSize);

/**
 *  \brief Sets the current pitch to \p p (in Hertz).
 *
 *  \param pitch The pitch in Hertz.
 */
TTS_API void p_tts_SetPitch(int pitch);

/**
 *  \brief Changes current character to \p v.
 *
 *  \param v Specifies the new character. Possibilities are: "Default", "Man",
 *  "OldMan", "OldWoman", "Boy", YoungGirl", "Robot", "Giant", "Dwarf", "Alien".
 *
 * \return On success returns 0. On error returns -1.
 */
TTS_API int p_tts_SetCharacter(const char v[]);

/**
 *  \brief Instructs the TTS library to collect the next set of audio samples.
 *  This function should be called repeatedly from the application main loop, or
 *  from the thread that initialized the library when using a multi-threaded system.
 *  The program should keep on calling the function as long as the return value is 1.
 *  If the return value is -1, it means that the buffer is full. If the library is being
 *  used in streaming mode, then the program must call \ref p_tts_FlushSamples
 *  after making sure that the collected samples were sent to the audio device.
 *
 *  \return Returns 1 if the function should be called again. Otherwise returns 0 if the
 *  TTS synthesis has finished, or -1 if the buffer is full.
 */
TTS_API int p_tts_DoNextStep(void);

/**
 *  \brief Starts the text-to-speech synthesis. On both a multi-tasking environment
 *  and a single threaded system. This function returns before the speech is finished.
 *  Use \ref p_tts_IsFinished to poll the status of the synthesis.
 *  On single threaded (main-loop) systems keep calling the \ref p_tts_DoNextStep function
 *  until speech synthesis is finished.
 *
 *  \param text Pointer to text input string.
 *  \param textLEN The size of the text input string in characters.
 *  \param format The format of input text.
 *
 *  \return 0 on success or -1 on error.
 */
TTS_API int p_tts_StartTTS(
                const char text[],
                unsigned short textLEN,
                enum P_TTS_FORMAT format);

/**
 *  \brief Stops the text-to-speech synthesis.
 *  In multi threading model, the synthesis thread is stopped.
 *  In single threading model, the state is changed so that any
 *  subsequent calls to p_tts_DoNextStep will not have any effect.
 *
 *  \return On success returns 0. On error returns -1.
 */
TTS_API int p_tts_StopTTS(void);




#ifdef __cplusplus
}
#endif

#endif  // PTTS_H_
