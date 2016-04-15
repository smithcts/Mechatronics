// Includes
#include "user_pb.h"
#include "bootloader_init.h"

// Forward declarations
void setup_exception_handlers(void);
int eeva_main(void);

//*****************************************************************************
extern "C" int pre_main_setup(void)
{
    // Warning: as of right now the library startup procedure (_start) hasn't ran yet, it's what
    // jumps to main().  This is called right before that.

    // If the button closest to the USB port is being pressed when the robot is powered on
    // then go into bootloader mode so user can upload new code.
    UserPushButton push_button(USER_PB_BOTTOM);
    if (push_button.read())
    {
        bootloader_init();
    }

    // Setup functions to print out messages if program crashes.
    // For example if there's a division by zero or memory access violation.
    setup_exception_handlers();

    return 0;
}
