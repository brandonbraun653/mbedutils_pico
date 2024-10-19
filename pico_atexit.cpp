/******************************************************************************
 *  File Name:
 *    pico_atexit.cpp
 *
 *  Description:
 *    RPI Pico specific atexit implementation
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/system.hpp>

namespace mb::system::atexit
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
  }


  bool registerCallback( mb::system::atexit::Callback &callback, uint32_t priority )
  {
    return true;
  }


  bool unregisterCallback( mb::system::atexit::Callback &callback )
  {
    return true;
  }


  void exit()
  {
  }

}  // namespace mb::system::atexit
