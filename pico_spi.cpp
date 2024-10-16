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
#include <mbedutils/assert.hpp>
#include <mbedutils/interfaces/spi_intf.hpp>
#include "hardware/spi.h"

namespace mb::hw::spi::intf
{
  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  static spi_inst_t *get_spi_instance( const Port_t port )
  {
    switch( port )
    {
      case 0:
        return spi0;

      case 1:
        return spi1;

      default:
        mbed_assert_always();
        return nullptr;
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void driver_setup()
  {
  }


  void driver_teardown()
  {
    spi_deinit( spi0 );
    spi_deinit( spi1 );
  }


  void init( const SpiConfig &config )
  {
    spi_inst_t *spi = get_spi_instance( config.port );

    spi_init( spi, config.speed );    // Defaults to master-mode
    spi_set_format( spi, config.width, static_cast<spi_cpol_t>( config.polarity ), static_cast<spi_cpha_t>( config.phase ),
                    static_cast<spi_order_t>( config.order ) );

    uint32_t act_clock = spi_get_baudrate( spi );
    if( act_clock != config.speed )
    {
      mbed_assert_continue_msg( false, "SPI clock speed mismatch: Req %d, Act ", config.speed, act_clock );
    }
  }


  void deinit( const Port_t port )
  {
    spi_inst_t *spi = get_spi_instance( port );
    spi_deinit( spi );
  }


  int write( const Port_t port, const void * data, const size_t length )
  {
    spi_inst_t *spi = get_spi_instance( port );
    return spi_write_blocking( spi, reinterpret_cast<const uint8_t*>( data ), length );
  }


  int read( const Port_t port, void * data, const size_t length )
  {
    spi_inst_t *spi = get_spi_instance( port );
    spi_read_blocking( spi, 0, reinterpret_cast<uint8_t*>( data ), length );
  }


  int transfer( const Port_t port, const void * tx, void * rx, const size_t length )
  {
    spi_inst_t *spi = get_spi_instance( port );
    return spi_write_read_blocking( spi, reinterpret_cast<const uint8_t*>( tx ), reinterpret_cast<uint8_t*>( rx ), length );
  }

}    // namespace mb::hw::spi::intf
