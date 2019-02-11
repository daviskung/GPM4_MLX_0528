#include "AudioDev.h"

void AUDIODEV_Init(void)
{
    // Do nothing.
}

void AUDIODEV_OpenPlayback(void)
{
    // Do nothing.
}

void AUDIODEV_SetVolume(uint32_t volume)
{
    // Do nothing.
}

void AUDIODEV_SetDataRate(uint32_t sampleRate, uint32_t format)
{
    // Do nothing.
}

void AUDIODEV_ClosePlayback(void)
{
    // Do nothing.
}

uint32_t AUDIODEV_WriteSamples(const void * data, uint32_t nrOfSamples)
{
    // Do nothing. Just pretend operation was successful.
    return nrOfSamples;
}

uint32_t AUDIODEV_GetNrOfQueuedSamples(void)
{
    // No samples in buffer.
    return 0;
}

uint32_t AUDIODEV_GetFreeSampleQueueSpace(void)
{
    // No audio buffer in the dummy device.
    return 0;
}

uint32_t AUDIODEV_GetSizeOfSampleQueue(void)
{
    // No audio buffer in the dummy device.
    return 0;
}

uint32_t AUDIODEV_GetPlayedSamples(void)
{
    // No samples are played.
    return 0;
}

void AUDIODEV_Pause()
{
    // Do nothing.
}

void AUDIODEV_Resume()
{
    // Do nothing.
}
