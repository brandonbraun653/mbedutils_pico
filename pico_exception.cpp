/******************************************************************************
 *  File Name:
 *    pico_exception.cpp
 *
 *  Description:
 *    Pico SDK exception handling interfaces
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/assert.hpp>
#include <mbedutils/interfaces/exception_intf.hpp>
#include <hardware/exception.h>

namespace mb::hw::exception::intf
{
  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  static void pico_hardfault_handler()
  {
    mbed_assert_always();
    while( 1 )
    {
      asm("nop");
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void driver_setup()
  {
    exception_set_exclusive_handler( HARDFAULT_EXCEPTION, pico_hardfault_handler );
  }

}  // namespace mb::hw::exception::intf
