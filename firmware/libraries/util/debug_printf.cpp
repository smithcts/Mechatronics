// Includes
#include <cstdio>
#include <cstring>
#include <cstdarg>

#include "telemetry_send_task.h"
#include "globs.h"
#include "debug_printf.h"
#include "usart.h"

//*****************************************************************************
void debug_printf
    (
        const char * format, // Message to print.
        ...                  // Variable arguments. (just like printf() uses)
    )
{
    glo_debug_message_t debug_message;

    va_list args;
    va_start(args, format);
    vsnprintf(debug_message.text, TELEMETRY_TEXT_SIZE, format, args);
    va_end(args);

    send_task.handle(debug_message);

}
