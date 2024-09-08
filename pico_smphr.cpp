/******************************************************************************
 *  File Name:
 *    pico_smphr.cpp
 *
 *  Description:
 *    RPI Pico SDK implementation of the semaphore interface. This is meant to
 *    be used in a bare metal application with no RTOS.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/array.h>
#include <limits>
#include <mbedutils/assert.hpp>
#include <mbedutils/config.hpp>
#include <mbedutils/interfaces/smphr_intf.hpp>
#include <pico/sync.h>

/* These drivers conflict with FreeRTOS based semaphores */
#if !__has_include( "FreeRTOS.h" )
namespace mb::osal
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

#if MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0
  using smphr_array = etl::array<semaphore_t, MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE>;
  static smphr_array        s_smphr_pool;
  static size_t             s_smphr_pool_index;
  static critical_section_t s_smphr_pool_cs;
#endif    // MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initSmphrDriver()
  {
#if MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0
    s_smphr_pool_index = 0;
    critical_section_init( &s_smphr_pool_cs );
#endif    // MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0
  }


  bool createSmphr( mb_smphr_t &s, const size_t maxCount, const size_t initialCount )
  {
    mbed_dbg_assert( s == nullptr );
    mbed_dbg_assert( maxCount < std::numeric_limits<int16_t>::max() );
    mbed_dbg_assert( initialCount < std::numeric_limits<int16_t>::max() );

    auto tmp = new semaphore_t();
    if( tmp == nullptr )
    {
      mbed_assert_continue_msg( false, "Failed to allocate semaphore" );
      return false;
    }

    sem_init( tmp, initialCount, maxCount );
    s = reinterpret_cast<mb_smphr_t>( tmp );
    return true;
  }


  void destroySmphr( mb_smphr_t &s )
  {
    mbed_dbg_assert( s != nullptr );
    delete static_cast<semaphore_t *>( s );
    s = nullptr;
  }


  bool allocateSmphr( mb_smphr_t &s, const size_t maxCount, const size_t initialCount )
  {
    bool allocated = false;
    mbed_dbg_assert( s == nullptr );
    mbed_dbg_assert( maxCount < std::numeric_limits<int16_t>::max() );
    mbed_dbg_assert( initialCount < std::numeric_limits<int16_t>::max() );

#if MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0
    critical_section_enter_blocking( &s_smphr_pool_cs );
    {
      if( s_smphr_pool_index < s_smphr_pool.size() )
      {
        sem_init( &s_smphr_pool[ s_smphr_pool_index ], initialCount, maxCount );
        s = reinterpret_cast<mb_smphr_t>( &s_smphr_pool[ s_smphr_pool_index ] );
        s_smphr_pool_index++;
        allocated = true;
      }
    }
    critical_section_exit( &s_smphr_pool_cs );
#endif    // MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0

    return allocated;
  }


  void deallocateSemaphore( mb_smphr_t &s )
  {
    // TODO: Implement when/if needed. Likely need to change to a pool allocator.
    mbed_dbg_assert( s != nullptr );
    mbed_assert_always();
  }


  size_t getSmphrAvailable( mb_smphr_t &s )
  {
    mbed_dbg_assert( s != nullptr );
    return static_cast<size_t>( sem_available( static_cast<semaphore_t *>( s ) ) );
  }


  void releaseSmphr( mb_smphr_t &s )
  {
    mbed_dbg_assert( s != nullptr );
    sem_release( static_cast<semaphore_t *>( s ) );
  }


  void releaseSmphrFromISR( mb_smphr_t &s )
  {
    /* Pico doesn't need to do anything fancy for this */
    mbed_dbg_assert( s != nullptr );
    sem_release( static_cast<semaphore_t *>( s ) );
  }


  void acquireSmphr( mb_smphr_t &s )
  {
    mbed_dbg_assert( s != nullptr );
    sem_acquire_blocking( static_cast<semaphore_t *>( s ) );
  }


  bool tryAcquireSmphr( mb_smphr_t &s )
  {
    mbed_dbg_assert( s != nullptr );
    return sem_acquire_timeout_ms( static_cast<semaphore_t *>( s ), 0 );
  }


  bool tryAcquireSmphr( mb_smphr_t &s, const size_t timeout )
  {
    mbed_dbg_assert( s != nullptr );
    mbed_dbg_assert( timeout < std::numeric_limits<uint32_t>::max() );
    return sem_acquire_timeout_ms( static_cast<semaphore_t *>( s ), static_cast<uint32_t>( timeout ) );
  }
}    // namespace mb::osal

#endif /* !__has_include( "FreeRTOS.h" ) */
