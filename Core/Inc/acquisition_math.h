#ifndef ACQUISITION_MATH_H
#define ACQUISITION_MATH_H
#include <stdint.h>

/* This loop must meet the DMA deadline even in the otherwise -O0 build. */
#if defined(__GNUC__) && !defined(__clang__)
#define ACQUISITION_FAST __attribute__((optimize("O2")))
#else
#define ACQUISITION_FAST
#endif

typedef struct
{
    uint32_t sum;
    uint32_t count;
    uint16_t peak;
} AcquisitionSamples;

/* Completed scans only; caller guarantees produced-consumed <= capacity. */
static inline ACQUISITION_FAST AcquisitionSamples AcquisitionAccumulate(
    const volatile uint16_t *buffer, uint32_t capacity, uint32_t ranks,
    uint32_t slot, uint32_t consumed, uint32_t produced, uint32_t first)
{
    AcquisitionSamples result = {0U, 0U, 0U};
    uint32_t pending = produced - consumed;
    if ((int32_t)(first - consumed) > 0)
    {
        uint32_t skip = first - consumed;
        if (skip >= pending) return result;
        consumed += skip;
        pending -= skip;
    }
    uint32_t scan_index = consumed % capacity;
    while (pending--)
    {
        uint16_t sample = buffer[scan_index * ranks + slot];
        result.sum += sample;
        result.count++;
        if (sample > result.peak) result.peak = sample;
        if (++scan_index == capacity) scan_index = 0U;
    }
    return result;
}

/* Cursor arithmetic is unsigned, including wrap of the lifetime scan counter. */
static inline uint32_t AcquisitionPending(uint32_t produced, uint32_t consumed)
{
    return produced - consumed;
}

static inline uint64_t AcquisitionProducedWords(uint32_t epochs,
    uint32_t remaining, uint32_t capacity, uint8_t tc_pending)
{
    epochs += tc_pending ? 1U : 0U;
    if (remaining == 0U && tc_pending) remaining = capacity;
    return (uint64_t)epochs * capacity + capacity - remaining;
}

static inline uint32_t AcquisitionFirstCompleteScan(uint32_t words, uint32_t ranks)
{
    /* Skip the in-flight scan at a rising edge. */
    return (words + ranks - 1U) / ranks;
}

static inline uint32_t AcquisitionCaptureTime(uint32_t epoch, uint16_t captured,
                                             uint16_t counter, uint8_t update_pending)
{
    /* A pending overflow belongs to the capture only if it preceded the edge. */
    if (update_pending && captured <= counter) epoch += 0x10000U;
    return epoch + captured;
}
#endif
