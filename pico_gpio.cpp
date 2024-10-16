/******************************************************************************
 *  File Name:
 *    pico_gpio.cpp
 *
 *  Description:
 *    Mbedutils GPIO driver with a Pico backend
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/array.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/interfaces/gpio_intf.hpp>
#include "hardware/gpio.h"

namespace mb::hw::gpio::intf
{
  /*---------------------------------------------------------------------------
  Private Data
  ---------------------------------------------------------------------------*/

  static etl::array<Callback_t, NUM_BANK0_GPIOS> s_callbacks;
  static uint32_t s_irq_mask;

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  static void gpio_irq_callback( uint gpio, uint32_t event_mask )
  {
    if( gpio < s_callbacks.size() && s_callbacks[ gpio ] )
    {
      s_callbacks[ gpio ]( event_mask );
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void driver_setup()
  {
    s_irq_mask = 0;
    s_callbacks.fill( {} );

    irq_set_enabled( IO_IRQ_BANK0, false );
  }


  void driver_teardown()
  {
    /*-------------------------------------------------------------------------
    Disable the interrupt line
    -------------------------------------------------------------------------*/
    irq_set_enabled( IO_IRQ_BANK0, false );

    /*-------------------------------------------------------------------------
    Disable all interrupts
    -------------------------------------------------------------------------*/
    s_irq_mask = 0;
    s_callbacks.fill( {} );

    for( uint32_t i = 0; i < NUM_BANK0_GPIOS; i++ )
    {
      gpio_set_irq_enabled( i, 0, false );
      gpio_deinit( i );
    }
  }


  bool init( const PinConfig &config )
  {


    gpio_init( config.pin );
    return false;
  }


  void write( const Port_t port, const Pin_t pin, const State_t state )
  {
    gpio_put( pin, state == State_t::STATE_HIGH );
  }


  void toggle( const Port_t port, const Pin_t pin )
  {
    gpio_put( pin, !gpio_get( pin ) );
  }


  State_t read( const Port_t port, const Pin_t pin )
  {
    return gpio_get( pin ) ? State_t::STATE_HIGH : State_t::STATE_LOW;
  }


  void setAlternate( const Port_t port, const Pin_t pin, const Alternate_t alternate )
  {
    gpio_set_function( pin, static_cast<gpio_function>( alternate ) );
  }


  void setPull( const Port_t port, const Pin_t pin, const Pull_t pull )
  {
    if( pull == Pull_t::PULL_UP )
    {
      gpio_set_pulls( pin, true, false );
    }
    else if( pull == Pull_t::PULL_DOWN )
    {
      gpio_set_pulls( pin, false, true );
    }
    else
    {
      gpio_set_pulls( pin, false, false );
    }
  }


  void setDrive( const Port_t port, const Pin_t pin, const Drive_t drive )
  {
    gpio_set_drive_strength( pin, static_cast<gpio_drive_strength>( drive ) );
  }


  void setSpeed( const Port_t port, const Pin_t pin, const Speed_t speed )
  {
    gpio_set_slew_rate( pin, static_cast<gpio_slew_rate>( speed ) );
  }


  void setMode( const Port_t port, const Pin_t pin, const Mode_t mode )
  {
    switch (mode)
    {
      case Mode_t::MODE_INPUT:
        gpio_set_input_enabled( pin, true );
        gpio_set_dir( pin, GPIO_IN );
        break;

      case Mode_t::MODE_OUTPUT:
        gpio_set_input_enabled( pin, false );
        gpio_set_dir( pin, GPIO_OUT );
        break;

      default:
        // Handle other modes if any
        mbed_assert_continue_msg( false, "Unsupported GPIO mode: %d", static_cast<int>( mode ) );
        break;
    }
  }


  void attachInterrupt( const Port_t port, const Pin_t pin, const Trigger_t trigger, const Callback_t &callback )
  {
    mbed_assert( pin < s_callbacks.size() );

    /*-------------------------------------------------------------------------
    Store the callback and pin interrupt status
    -------------------------------------------------------------------------*/
    s_callbacks[ pin ] = callback;
    s_irq_mask |= ( 1u << pin );

    /*-------------------------------------------------------------------------
    Configure the interrupt
    -------------------------------------------------------------------------*/
    gpio_set_irq_enabled( pin, trigger, true );
    gpio_set_irq_callback( gpio_irq_callback );
    irq_set_enabled( IO_IRQ_BANK0, true );
  }


  void detachInterrupt( const Port_t port, const Pin_t pin )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( pin >= s_callbacks.size() )
    {
      mbed_assert_continue_always();
      return;
    }

    /*-------------------------------------------------------------------------
    Clear the interrupt configuration
    -------------------------------------------------------------------------*/
    gpio_set_irq_enabled( pin, 0, false );
    s_callbacks[ pin ] = {};
    s_irq_mask &= ~( 1u << pin );

    /*-------------------------------------------------------------------------
    Disable the interrupt if no more pins are using it
    -------------------------------------------------------------------------*/
    if( s_irq_mask == 0 )
    {
      irq_set_enabled( IO_IRQ_BANK0, false );
    }
  }


  uint32_t getInterruptLine( const Port_t port, const Pin_t pin )
  {
    // Pretty sure this is just the pin number?
    return pin;
  }


}  // namespace mb::hw::gpio::intf
