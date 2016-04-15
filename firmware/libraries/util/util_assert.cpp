// Includes
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "telemetry_send_task.h"
#include "debug_printf.h"
#include "util_assert.h"
#include "globs.h"


/*****************************************************************************
* Function: util_assert_failed
*
* Description:  Called whenever an assert fails.  Logs assert message or
*               condition (if no explicit message provided) and then performs
*               the action specified.  The variable argument (...) are provided
*               so this function can be used similar to printf().
*
* Notes: A newline character is automatically appended to format message string.
*****************************************************************************/
void util_assert_failed
    (
        int action,             // What to do after logging.  Such as restarting system.
        char const * file_name, // File name that assert failed in.
        int line_number,        // Line number that assert failed on.
        const char * format,    // Message to log.
        ...                     // Variable arguments. (just like printf() uses)
    )
{
    const int buffer_size = TELEMETRY_TEXT_SIZE;
    char buffer[buffer_size];
    int current_index = 0;
    char const * just_file_name = NULL;

    // If line number is zero then meta logging is disabled.
    // Otherwise need to take it into account.
    if (line_number >= 0)
    {
        // By default __FILE__ expands to full filename path given to compiler.
        // Split path to make sure only get filename and extension.
        just_file_name = strrchr(file_name, '/');

        if (just_file_name == NULL)
        {
            just_file_name = strrchr(file_name, '\\');

            if (just_file_name == NULL)
            {
                just_file_name = file_name;
            }
        }

        current_index += snprintf(buffer, buffer_size, "File: %s %d: ", just_file_name, line_number);
    }

    // Copy variable arguments into buffer.
    va_list args;
    va_start(args, format);
    current_index += vsnprintf(buffer + current_index, buffer_size - current_index, format, args);
    current_index += 2; // Advance to next spot.
    va_end(args);

    // Need to make sure have at least three spots for carriage return, newline, and nul.
    // Only need to check '-2' instead of '-3' since existing nul from vsnprintf is saving
    // us one spot already.
    if (current_index >= buffer_size - 2)
    {
        current_index = buffer_size - 2;
    }

    // Copy newline character automatically since all asserts should be independent.
    buffer[current_index-2] = '\r'; //
    buffer[current_index-1] = '\n'; // Replace existing nul character with newline.
    buffer[current_index]   = '\0'; // Re-establish nul character to terminate string.

    glo_assert_message_t assert_message;
    assert_message.action = action;
    memcpy(assert_message.text, buffer, strlen(buffer) + 1); // plus 1 for nul character

    send_task.handle(assert_message);

    // Right restart asserts are treated the same as stop asserts.
    if ((action == ASSERT_STOP) || (action == ASSERT_RESTART))
    {
        // Since the send task didn't throw an assert we should be able to send back the assert message.
        // Need to go through scheduler so it can update which task is running in case send task throws
        // an assert when it's being flushed.
        if (scheduler.runningTaskID() != TASK_ID_TELEM_SEND)
        {
            scheduler.flushOutgoingMessages();

            // Since the receive task didn't throw an assert we should be able to wait for the UI to connect
            // (if it's not already) and request the most recent assert message.
            if (scheduler.runningTaskID() != TASK_ID_TELEM_RECEIVE)
            {
                while (true)
                {
                    // Every so often check if got any new requests so user can get the cause of the assert.
                    // Send back meaningless status update as a 'heartbeat' for UI.
                    for (uint32_t i = 0; i < 3e5; ++i)
                    {
                        asm(""); // Assert failed! That's why we're here.
                    }
                    send_task.send(GLO_ID_STATUS_DATA);
                    scheduler.flushIncomingMessages();
                    scheduler.flushOutgoingMessages();
                }
            }
        }

        while (true) {}; // Assert failed!
    }

}
