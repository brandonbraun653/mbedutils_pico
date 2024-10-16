/******************************************************************************
 *  File Name:
 *    pico_spi.cpp
 *
 *  Description:
 *    Mbedutils SPI driver with a Pico backend
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/interfaces/spi_intf.hpp>
#include "hardware/spi.h"

namespace mb::hw::spi::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void init( const SpiConfig &config )
  {
  }


  void deinit( const Port_t port )
  {
  }


  void write( const Port_t port, const void *const data, const size_t length )
  {
  }


  void read( const Port_t port, void *const data, const size_t length )
  {
  }


  void transfer( const Port_t port, const void *const tx, void *const rx, const size_t length )
  {
  }


  void lock( const Port_t port )
  {
  }


  void unlock( const Port_t port )
  {
  }

}  // namespace mb::hw::spi::intf
