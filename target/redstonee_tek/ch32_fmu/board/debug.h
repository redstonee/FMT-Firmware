#pragma once

void initDebugUART(void);

void debugUARTSend(const char* fmt, ...);

#define debug_print(fmt, ...) debugUARTSend("%s(): %d " fmt "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)