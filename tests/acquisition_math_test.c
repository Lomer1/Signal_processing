#include <assert.h>
#include <stdio.h>
#include "acquisition_math.h"

int main(void)
{
    /* Pending TC and serviced TC must identify the same DMA position. */
    assert(AcquisitionProducedWords(0, 308, 1024, 1) == 1740);
    assert(AcquisitionProducedWords(1, 308, 1024, 0) == 1740);
    assert(AcquisitionProducedWords(0, 0, 1024, 1) == 1024);
    assert(AcquisitionProducedWords(1, 1024, 1024, 0) == 1024);
    /* Repeated HT/TC boundaries, then a fresh burst with counters reset. */
    for (uint32_t epoch = 0; epoch < 1000; ++epoch)
    {
        uint32_t half = (uint32_t)(AcquisitionProducedWords(epoch, 512, 1024, 0) / 4);
        uint32_t full = (uint32_t)(AcquisitionProducedWords(epoch, 1024, 1024, 1) / 4);
        assert(AcquisitionPending(full, half) == 128);
        assert(AcquisitionProducedWords(epoch + 1, 1024, 1024, 0) / 4 == full);
    }
    assert(AcquisitionProducedWords(0, 1024, 1024, 0) == 0);
    /* A missing wrap must remain a fault, never be clamped to zero. */
    assert(AcquisitionPending(381, 435) == UINT32_MAX - 53);
    uint16_t ring[8 * 4];
    for (unsigned i = 0; i < 8; ++i)
        for (unsigned slot = 0; slot < 4; ++slot)
            ring[i * 4 + slot] = (uint16_t)(100 * (slot + 1));
    AcquisitionSamples a = AcquisitionAccumulate(ring, 8, 4, 0, 0, 5, 0);
    assert(a.count == 5 && a.sum == 500 && a.peak == 100);
    /* A second pulse with changed amplitude must not reuse the first result. */
    for (unsigned i = 0; i < 8; ++i) ring[i * 4] = 750;
    a = AcquisitionAccumulate(ring, 8, 4, 0, 0, 5, 0);
    assert(a.count == 5 && a.sum == 3750 && a.peak == 750);
    /* Overlapping source starts later; exclude pre-pulse samples. */
    a = AcquisitionAccumulate(ring, 8, 4, 1, 0, 5, 3);
    assert(a.count == 2 && a.sum == 400);
    a = AcquisitionAccumulate(ring, 8, 4, 1, 0, 5, 5);
    assert(a.count == 0);
    /* Circular wrap, chunk splitting and lifetime counter wrap. */
    a = AcquisitionAccumulate(ring, 8, 4, 2, 6, 10, 6);
    assert(a.count == 4 && a.sum == 1200);
    AcquisitionSamples b = AcquisitionAccumulate(ring, 8, 4, 2, 6, 8, 6);
    AcquisitionSamples c = AcquisitionAccumulate(ring, 8, 4, 2, 8, 10, 6);
    assert(a.sum == b.sum + c.sum && a.count == b.count + c.count);
    a = AcquisitionAccumulate(ring, 8, 4, 3, UINT32_MAX - 1, 2, UINT32_MAX);
    assert(a.count == 3 && a.sum == 1200);
    assert(AcquisitionPending(2, UINT32_MAX - 1) == 4);
    assert(AcquisitionCaptureTime(65536, 65530, 5, 1) == 131066);
    assert(AcquisitionCaptureTime(65536, 3, 5, 1) == 131075);
    puts("acquisition math: PASS");
    return 0;
}
