#pragma once
#include <stdint.h>
#include "AllocationResponseObserver.h"
// Setup/HTTP task only. No control-loop waits or heap walks are introduced.
void initializeUartAllocationDiagnostics();
void sampleAllocationDiagnosticHeap(uint32_t ms,uint32_t free,uint32_t largest);

void beginAllocationResponse();
AllocationResponseSummary finishAllocationResponse();
