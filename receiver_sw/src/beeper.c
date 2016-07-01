#include "beeper.h"
#include <ch.h>
#include <hal.h>
#include "hal_lld.h"
#include <stm32l1xx.h>
#include <stdlib.h>
#include "stm32_dma.h"

//#include <../os/ext/CMSIS/ST/STM32L1xx/stm32l152xc.h>

void beeper_init()
{
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR |= DAC_CR_EN2;
    DAC->DHR12R2 = 0;
}

void beeper_deinit()
{
    DAC->CR &= ~DAC_CR_EN2;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
}

void beeper_click()
{
    DAC->DHR12R2 = 2048; //4095;
    chThdSleepMilliseconds(2);
    DAC->DHR12R2 = 0;
}

#define BUFSIZE 64
static uint16_t g_buffer[BUFSIZE];

static void smoothen(int envelope)
{
    int v = g_buffer[BUFSIZE - 1];
    for (int i = 0; i < BUFSIZE; i++)
    {
        g_buffer[i] = (g_buffer[i] * 89 + v * 10) * envelope / 100000;
        v = g_buffer[i];
    }
}

void beeper_bling(int freq, int decay, int length, int volume)
{
    if (volume > 100) volume = 100;
    
    /* Initialize with randomness */
    for (int i = 0; i < BUFSIZE; i++)
    {
        g_buffer[i] = ((i * 31337) & 4095) * volume / 100;
    }
    
    /* Use DMA to feed the samples to DAC */
    const stm32_dma_stream_t *stream = STM32_DMA1_STREAM3;
    dmaStreamAllocate(stream, 10, NULL, NULL);
    dmaStreamSetPeripheral(stream, &DAC->DHR12R2);
    dmaStreamSetMemory0(stream, g_buffer);
    dmaStreamSetTransactionSize(stream, BUFSIZE);
    dmaStreamSetMode(stream, STM32_DMA_CR_PL(0) | STM32_DMA_CR_DIR_M2P | \
                             STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD | \
                             STM32_DMA_CR_MINC | \
                             STM32_DMA_CR_CIRC);
    
    dmaStreamEnable(stream);
    
    /* Sound frequency is determined by buffer length and samplerate */
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 0;
    TIM7->ARR = (STM32_TIMCLK1 / (freq * BUFSIZE)) - 1;
    TIM7->DIER = TIM_DIER_UDE;
    TIM7->CR1 = TIM_CR1_CEN;
    
    /* Iteratively smoothen the buffer contents to have a guitarlike decaying
     * sound. */
    uint32_t endtime = chTimeNow() + length;
    while (chTimeNow() < endtime)
    {
        chThdSleepMilliseconds(5);
        int envelope = (endtime - chTimeNow()) * 1000 / decay;
        if (envelope > 1000) envelope = 1000;
        smoothen(envelope);
    }
    
    /* Finish up and end in a low-power state */
    TIM7->CR1 = 0;
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
    dmaStreamDisable(stream);
    dmaStreamRelease(stream);
    DAC->DHR12R2 = 0;
}

typedef struct {
    uint16_t timedelta;
    uint16_t freq;
    uint8_t volume;
} note_t;

#define NUMNOTES 333
static const note_t notes[NUMNOTES];

void beeper_midi(keep_going_t callback)
{
    for (int i = 0; i < NUMNOTES; i++)
    {
        beeper_bling(notes[i].freq, 50, notes[i].timedelta, notes[i].volume);
        
        if (!callback())
            return;
    }
}

static const note_t notes[NUMNOTES] = 
{
{   192,  246,   95}, {   192,  329,   95}, {     8,  329,    0}, {   192,  440,   95},
{   192,  493,   95}, {     8,  493,    0}, {   192,  440,   95}, {   192,  246,   95},
{     8,  246,    0}, {   192,  329,   95}, {     8,  329,    0}, {   192,  246,   95},
{   200,  246,    0}, {   192,  329,   95}, {   192,  440,   95}, {     8,  440,    0},
{   192,  493,   95}, {     8,  493,    0}, {   192,  440,   95}, {   192,  329,   95},
{     8,  329,    0}, {   192,  246,   95}, {   192,  329,   95}, {   200,  329,    0},
{   192,  329,   95}, {     8,  329,    0}, {   192,  440,   95}, {     8,  440,    0},
{   192,  493,   95}, {   192,  440,   95}, {     8,  440,    0}, {   192,  246,   95},
{   192,  329,   95}, {     8,  329,    0}, {   192,  246,   95}, {   200,  246,    0},
{   192,  440,   95}, {     8,  440,    0}, {   192,  587,   95}, {   192,  523,   95},
{     8,  523,    0}, {   192,  587,   95}, {   192,  523,   95}, {     8,  523,    0},
{   192,  493,   95}, {     8,  493,    0}, {   192,  440,   95}, {   384,  493,   95},
{   200,  493,    0}, {   976,  493,   95}, {     8,  493,    0}, {   392,  493,   95},
{   200,  493,    0}, {   392,  440,   95}, {   576,  587,   95}, {   392,  493,   95},
{   200,  493,    0}, {   976,  493,   95}, {     8,  493,    0}, {   392,  493,   95},
{  1168,  440,   95}, {   192,  523,   88}, {     8,  523,    0}, {   192,  523,   88},
{     8,  523,    0}, {   192,  523,   88}, {   192,  523,   88}, {     8,  523,    0},
{   192,  523,   88}, {   192,  523,   88}, {     8,  523,    0}, {   192,  523,   88},
{     8,  523,    0}, {   192,  523,   88}, {   784,  493,   95}, {   192,  369,   95},
{     8,  369,    0}, {   392,  440,   95}, {   976,  391,   95}, {     8,  391,    0},
{   192,  523,   95}, {     8,  523,    0}, {   392,  523,   95}, {   392,  493,   95},
{   192,  523,   95}, {     8,  523,    0}, {   192,  493,   95}, {   192,  440,   95},
{     8,  440,    0}, {   392,  391,   95}, {   392,  659,   95}, {   784,  440,   95},
{     8,  440,    0}, {   192,  523,   95}, {   392,  523,   95}, {   968,  493,   95},
{     8,  493,    0}, {   192,  369,   95}, {     8,  369,    0}, {   392,  440,   95},
{   976,  391,   95}, {     8,  391,    0}, {   192,  523,   95}, {   392,  523,   95},
{   392,  493,   95}, {     8,  493,    0}, {   192,  523,   95}, {   192,  493,   95},
{     8,  493,    0}, {   192,  440,   95}, {   392,  391,   95}, {     8,  391,    0},
{   392,  659,   95}, {   784,  440,   95}, {   784,  493,   95}, {   192,  523,   95},
{     8,  523,    0}, {   192,  493,   95}, {     8,  493,    0}, {   192,  440,   95},
{   392,  523,   95}, {   192,  493,   95}, {     8,  493,    0}, {   192,  440,   95},
{     8,  440,    0}, {   392,  587,   95}, {   192,  523,   95}, {   192,  493,   95},
{     8,  493,    0}, {   392,  587,   95}, {   192,  523,   95}, {     8,  523,    0},
{   192,  493,   95}, {   392,  659,   95}, {   192,  587,   95}, {     8,  587,    0},
{   192,  659,   95}, {     8,  659,    0}, {   392,  739,   95}, {   392,  493,   95},
{   584,  783,   95}, {     8,  783,    0}, {   392,  739,   95}, {   392,  698,   95},
{   392,  493,   95}, {   192,  783,   95}, {     8,  783,    0}, {   192,  659,   95},
{   192,  493,   95}, {     8,  493,    0}, {   192,  739,   95}, {     8,  739,    0},
{   192,  587,   95}, {   192,  440,   95}, {     8,  440,    0}, {   192,  659,   95},
{   192,  523,   95}, {     8,  523,    0}, {   192,  391,   95}, {   192,  587,   95},
{     8,  587,    0}, {   192,  493,   95}, {     8,  493,    0}, {   192,  391,   95},
{   192,  523,   95}, {     8,  523,    0}, {   192,  329,   95}, {   192,  493,   95},
{     8,  493,    0}, {   192,  293,   95}, {     8,  293,    0}, {   192,  523,   95},
{   192,  493,   95}, {     8,  493,    0}, {   192,  440,   95}, {   192,  391,   95},
{     8,  391,    0}, {   392,  466,   95}, {   392,  440,   95}, {   192,  523,   88},
{     8,  523,    0}, {   192,  523,   88}, {   192,  523,   88}, {     8,  523,    0},
{   192,  523,   88}, {     8,  523,    0}, {   192,  523,   88}, {   192,  523,   88},
{     8,  523,    0}, {   192,  523,   88}, {   192,  523,   88}, {     8,  523,    0},
{   784,  493,   95}, {   192,  369,   95}, {     8,  369,    0}, {   392,  440,   95},
{   976,  391,   95}, {     8,  391,    0}, {   192,  523,   95}, {     8,  523,    0},
{   392,  523,   95}, {   392,  493,   95}, {   192,  523,   95}, {   192,  493,   95},
{     8,  493,    0}, {   192,  440,   95}, {     8,  440,    0}, {   392,  391,   95},
{   392,  659,   95}, {   784,  440,   95}, {   192,  523,   95}, {     8,  523,    0},
{   392,  523,   95}, {   976,  493,   95}, {     8,  493,    0}, {   192,  369,   95},
{     8,  369,    0}, {   392,  440,   95}, {   976,  391,   95}, {     8,  391,    0},
{   192,  523,   95}, {   392,  523,   95}, {   392,  493,   95}, {     8,  493,    0},
{   192,  523,   95}, {   192,  493,   95}, {     8,  493,    0}, {   192,  440,   95},
{   392,  391,   95}, {     8,  391,    0}, {   392,  659,   95}, {   784,  440,   95},
{   784,  493,   95}, {   192,  523,   95}, {     8,  523,    0}, {   192,  493,   95},
{     8,  493,    0}, {   192,  440,   95}, {   392,  523,   95}, {   192,  493,   95},
{     8,  493,    0}, {   192,  440,   95}, {   392,  587,   95}, {     8,  587,    0},
{   192,  523,   95}, {   192,  493,   95}, {     8,  493,    0}, {   392,  587,   95},
{   192,  523,   95}, {     8,  523,    0}, {   192,  493,   95}, {   392,  659,   95},
{   192,  587,   95}, {     8,  587,    0}, {   192,  659,   95}, {   392,  739,   95},
{     8,  739,    0}, {   392,  493,   95}, {   584,  783,   95}, {     8,  783,    0},
{   392,  739,   95}, {   392,  698,   95}, {   392,  493,   95}, {   192,  783,   95},
{     8,  783,    0}, {   192,  659,   95}, {   192,  493,   95}, {     8,  493,    0},
{   192,  739,   95}, {   192,  587,   95}, {     8,  587,    0}, {   192,  440,   95},
{     8,  440,    0}, {   192,  659,   95}, {   192,  523,   95}, {     8,  523,    0},
{   192,  391,   95}, {   192,  587,   95}, {     8,  587,    0}, {   192,  493,   95},
{     8,  493,    0}, {   192,  391,   95}, {   192,  523,   95}, {     8,  523,    0},
{   192,  329,   95}, {   192,  493,   95}, {     8,  493,    0}, {   192,  293,   95},
{   192,  523,   95}, {     8,  523,    0}, {   192,  493,   95}, {     8,  493,    0},
{   192,  440,   95}, {   192,  391,   95}, {     8,  391,    0}, {   384,  466,   95},
{   392,  440,   95}, {   192,  783,   95}, {     8,  783,    0}, {   192,  391,   95},
{   192,  587,   95}, {     8,  587,    0}, {   192,  391,   95}, {   192,  622,   95},
{     8,  622,    0}, {   192,  311,   95}, {   192,  466,   95}, {     8,  466,    0},
{   192,  440,   95}, {     8,  440,    0}, {   192,  391,   95}, {   192,  195,   95},
{     8,  195,    0}, {   192,  293,   95}, {   192,  195,   95}, {     8,  195,    0},
{   192,  311,   95}, {     8,  311,    0}, {   192,  195,   95}, {   192,  233,   95},
{     8,  233,    0}, {   192,  220,   95}, {   192,  195,   88}, {     8,  195,    0},
{   192,  195,   90}, {     8,  195,    0}, {   192,  195,   91}, {   192,  195,   93},
{     8,  195,    0}, {   192,  195,   95}, {   192,  195,  101}, {     8,  195,    0},
{   192,  195,  107}
    
};
    
    










