/******************************************************************************
 *  File Name:
 *    pico_system.cpp
 *
 *  Description:
 *    RPI Pico SDK implementation of the system interface
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/interfaces/system_intf.hpp>
#include "hardware/watchdog.h"

namespace mb::system::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void warm_reset()
  {
    // Reboot the Raspberry Pi Pico using the watchdog
    watchdog_enable( 0, 1 );
    watchdog_reboot( 0, 0, 0 );

    while( true )
    {
      tight_loop_contents();    // Wait for the reboot to occur
    }
  }
}    // namespace mb::system::intf
