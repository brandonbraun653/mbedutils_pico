/******************************************************************************
 *  File Name:
 *    pico_mutex.cpp
 *
 *  Description:
 *    RPI Pico SDK implementation of the mutex interface. This is meant to be
 *    used in a bare metal application with no RTOS.
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
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <pico/sync.h>

/* These drivers conflict with FreeRTOS based mutexes */
#if !__has_include( "FreeRTOS.h" )
namespace mb::osal
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
  using mt_array = etl::array<mutex_t, MBEDUTILS_OSAL_MUTEX_POOL_SIZE>;
  static mt_array           s_mutex_pool;
  static size_t             s_mutex_pool_index;
  static critical_section_t s_mutex_pool_cs;
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
  using rmt_array = etl::array<recursive_mutex_t, MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE>;
  static rmt_array          s_r_mutex_pool;
  static size_t             s_r_mtx_pool_idx;
  static critical_section_t s_r_mtx_pool_cs;
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initMutexDriver()
  {
#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
    s_mutex_pool_index = 0;
    critical_section_init( &s_mutex_pool_cs );
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
    s_r_mtx_pool_idx = 0;
    critical_section_init( &s_r_mtx_pool_cs );
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
  }


  bool createMutex( mb_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex == nullptr );
    auto tmp = new mutex_t();
    if( tmp == nullptr )
    {
      mbed_assert_continue_msg( false, "Failed to allocate mutex" );
      return false;
    }

    mutex_init( tmp );
    mutex = reinterpret_cast<mb_mutex_t>( tmp );
    return true;
  }


  void destroyMutex( mb_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    delete static_cast<mutex_t *>( mutex );
    mutex = nullptr;
  }


  bool allocateMutex( mb_mutex_t &mutex )
  {
    bool allocated = false;
    mbed_dbg_assert( mutex == nullptr );

#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
    critical_section_enter_blocking( &s_mutex_pool_cs );
    {
      if( s_mutex_pool_index < s_mutex_pool.size() )
      {
        mutex_init( &s_mutex_pool[ s_mutex_pool_index ] );
        mutex = reinterpret_cast<mb_mutex_t>( &s_mutex_pool[ s_mutex_pool_index ] );
        s_mutex_pool_index++;
        allocated = true;
      }
    }
    critical_section_exit( &s_mutex_pool_cs );
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

    return allocated;
  }


  void deallocateMutex( mb_mutex_t &mutex )
  {
    // TODO: Implement when/if needed. Likely need to change to a pool allocator.
    mbed_dbg_assert( mutex != nullptr );
    mbed_assert_always();
  }


  void lockMutex( mb_mutex_t mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    mutex_enter_blocking( static_cast<mutex_t *>( mutex ) );
  }


  bool tryLockMutex( mb_mutex_t mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    return mutex_try_enter( static_cast<mutex_t *>( mutex ), nullptr );
  }


  bool tryLockMutex( mb_mutex_t mutex, const size_t timeout )
  {
    mbed_dbg_assert( mutex != nullptr );
    mbed_dbg_assert( timeout < std::numeric_limits<uint32_t>::max() );
    return mutex_enter_timeout_ms( static_cast<mutex_t *>( mutex ), static_cast<uint32_t>( timeout ) );
  }


  void unlockMutex( mb_mutex_t mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    mutex_exit( static_cast<mutex_t *>( mutex ) );
  }


  bool createRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex == nullptr );
    auto tmp = new recursive_mutex_t();
    if( tmp == nullptr )
    {
      mbed_assert_continue_msg( false, "Failed to allocate recursive mutex" );
      return false;
    }

    recursive_mutex_init( tmp );
    mutex = reinterpret_cast<mb_recursive_mutex_t>( tmp );
    return true;
  }


  void destroyRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    delete static_cast<recursive_mutex_t *>( mutex );
    mutex = nullptr;
  }


  bool allocateRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    bool allocated = false;
    mbed_dbg_assert( mutex == nullptr );

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
    critical_section_enter_blocking( &s_r_mtx_pool_cs );
    {
      if( s_r_mtx_pool_idx < s_r_mutex_pool.size() )
      {
        recursive_mutex_init( &s_r_mutex_pool[ s_r_mtx_pool_idx ] );
        mutex = reinterpret_cast<mb_recursive_mutex_t>( &s_r_mutex_pool[ s_r_mtx_pool_idx ] );
        s_r_mtx_pool_idx++;
        allocated = true;
      }
    }
    critical_section_exit( &s_r_mtx_pool_cs );
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0

    return allocated;
  }


  void lockRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    recursive_mutex_enter_blocking( static_cast<recursive_mutex_t *>( mutex ) );
  }


  bool tryLockRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    return recursive_mutex_try_enter( static_cast<recursive_mutex_t *>( mutex ), nullptr );
  }


  bool tryLockRecursiveMutex( mb_recursive_mutex_t mutex, const size_t timeout )
  {
    mbed_dbg_assert( mutex != nullptr );
    mbed_dbg_assert( timeout < std::numeric_limits<uint32_t>::max() );
    return recursive_mutex_enter_timeout_ms( static_cast<recursive_mutex_t *>( mutex ), static_cast<uint32_t>( timeout ) );
  }


  void unlockRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    recursive_mutex_exit( static_cast<recursive_mutex_t *>( mutex ) );
  }
}    // namespace mb::osal

#endif /* !has_include( "FreeRTOS.h" ) */
