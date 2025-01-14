/******************************************************************************
 *  File Name:
 *    pico_time.cpp
 *
 *  Description:
 *    RPI Pico implementation of the mb time interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/interfaces/time_intf.hpp>
#include "pico/time.h"

/* These drivers conflict with FreeRTOS based mutexes */
#if !__has_include( "FreeRTOS.h" )

namespace mb::time
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  int64_t millis()
  {
    return static_cast<int64_t>( to_ms_since_boot( get_absolute_time() ) );
  }


  int64_t micros()
  {
    return static_cast<int64_t>( to_us_since_boot( get_absolute_time() ) );
  }


  void delayMilliseconds( const size_t val )
  {
    sleep_us( val * 1000 );
  }


  void delayMicroseconds( const size_t val )
  {
    sleep_us( val );
  }

}    // namespace mb::time

#endif /* !__has_include( "FreeRTOS.h" ) */
