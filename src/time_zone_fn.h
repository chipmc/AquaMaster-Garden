#ifndef TIME_ZONE_FN_H
#define TIME_ZONE_FN_H

#include "PublishQueueAsyncRK.h"                    // Async Particle Publish

extern PublishQueueAsync publishQueue;

extern char currentOffsetStr[10];

int setTimeZone(String command);

int setDSTOffset(String command);

bool isDSTusa();

bool isDSTnz();

#endif
