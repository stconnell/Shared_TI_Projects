/*
 * Copyright (c) 2013-2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */
/*
 *  ======== sntp.h ========
 */
/**
 *  @file       sntp.h
 *
 *  The SNTP client uses the Network Time Protocol (NTP) to provide a continuous
 *  service of maintaining system time by periodically sychronizing with one or
 *  more NTP servers.
 *
 *  # Prerequisites For Use #
 *
 *  To implement the SNTP client service, the user is required to provide two
 *  system time keeping functions, as well as a list of NTP server IP addresses.
 *
 *  Furthermore, the SNTP client has been written to conform to the NDK BSD
 *  sockets support layer.  As such, it is required that the file containing any
 *  calls to the SNTP client APIs conform to the NDK BSD layer requirements.
 *
 *  Additionally, it is necessary to configure the automatic calling to
 *  fdOpenSession() and fdCloseSession() in an application which uses the SNTP
 *  client. This is accomplished by setting the configuration parameter
 *  "Global.autoOpenCloseFD = true;".
 *
 *  Further details on this configuration parameter and the NDK BSD layer
 *  requirements can be found in the NDK
 *  User's Guide and NDK API Guide.
 *
 *  # SNTP Client Time Functions #
 *
 *  The required time keeping functions are called by the SNTP client in order
 *  to get or set the current time in the system ("gettime" and "settime").
 *  For example, these could be APIs which set or get the value of a Real Time
 *  Clock (RTC), such as the SYS/BIOS Seconds module's Seconds_get() and
 *  Seconds_set() APIs. But, any equivalent API can be used.
 *
 *  The "gettime" and "settime" functions should have the
 *  following format and should be passed into the SNTP_start() function:
 *  @code
 *  uint32_t (*GetTimeFxn)(void);
 *  void (*SetTimeFxn)(uint32_t newtime);
 *  @endcode
 *
 *  The "gettime" function should return the number of seconds since the Epoch
 *  (January 1, 1970).  The "settime" function sets the time to the value
 *  that is passed to it.  The SNTP client uses the "gettime" function to write
 *  the current system time into a request to an NTP server. The "settime"
 *  function is used to set the system time to a new value received from the NTP
 *  server.  The SNTP client handles the conversion of NTP time (time since
 *  January 1, 1900) to the local system's time (number of seconds since January
 *  1, 1970).
 *
 *  The list of NTP servers should consist of socket address structures which
 *  contain the unicast addresses of the servers (as well as some additional
 *  information). Both IPv4 and IPv6 unicast
 *  server addresses are supported (host names are not supported).  Structures
 *  of type 'struct sockaddr_in' must be used for IPv4 server addresses and of
 *  type 'struct
 *  sockaddr_in6' for IPv6 server addresses. These structures must be written
 *  to a contiguous block of memory and then passed to the SNTP_start()
 *  function.
 *
 *  # General Usage #
 *
 *  The SNTP client may be stopped by calling SNTP_stop(). Calling this
 *  function will cause the SNTP client Task to exit and all of the resources
 *  used by it to be freed.
 *
 *  It is also possible to "force" a time synchronization with the NTP server.
 *  This may be achieved by calling the API SNTP_forceTimeSync().  This
 *  function will wake up the SNTP client and initiate communication with the
 *  current NTP server. Additionally, if this function is called, the SNTP
 *  client will call a user defined callback function (if configured) as soon
 *  as a successful time update from an NTP server has occurred.  This callback
 *  is passed as an argument to the SNTP_start() API.
 *
 *  Important note on the usage of SNTP_forceTimeSync().  The NTP protocol
 *  clearly specifies that an SNTP client must never send requests to an NTP
 *  server in intervals less than 15 seconds.  It is important to respect this
 *  NTP requirement and it is the user's responsibilty to ensure that
 *  SNTP_forceTimeSync() is not called more than once in any 15 second time
 *  period.
 *
 *  # Example Usage #
 *
 *  The SNTP module is part of the NDK nettools library and uses the NDK BSD
 *  support layer.  Therefore, an application must ensure that the following
 *  search path is passed to the compiler:
 *
 *      <NDK installation directory>/ti/ndk/inc/bsd
 *
 *  The application file which calls the SNTP APIs should comply to the NDK BSD
 *  layer as well.  Please refer to the NDK User's Guide and API guide for
 *  details on BSD sockets NDK applications.
 *
 *  To use the SNTP client APIs, the application should include its header file
 *  as follows:
 *  @code
 *  #include <ti/ndk/nettools/sntp/sntp.h>
 *  @endcode
 *
 *  ## Initializing The List Of Servers Using Socket Address Structures ##
 *
 *  The server list should be initialized and passed to the SNTP module via the
 *  SNTP_start() API (The list may be updated at a later point in the program
 *  by using the SNTP_setServers() API).
 *
 *  The following code uses socket address structs to
 *  store the IPv4 and IPv6 addresses of some (imaginary) NTP servers.  Note
 *  that it is no longer necessary to set the size of the address structure via
 *  the sin_len and sin6_len fields of the socket address structures.  When the
 *  length fields were removed from the definition of the socket address
 *  structures in NDK 2.24, the SNTP module was updated to handle cycling
 *  through the server_list based on the family type of the current address
 *  structure in the list.
 *
 *  Therefore, the family, port, IP address and scope ID (for IPv6 only) must
 *  be correctly set:
 *  @code
 *  #include <sys/socket.h>
 *
 *  struct sockaddr_in  ipv4addr;
 *  struct sockaddr_in6 ipv6addr;
 *
 *  ipv4addr.sin_family = AF_INET;
 *  ipv4addr.sin_port = htons(123);
 *  inet_pton(AF_INET, "192.168.1.100", &(ipv4addr.sin_addr.s_addr));
 *
 *  ipv6addr.sin6_family = AF_INET6;
 *  ipv6addr.sin6_port = htons(123);
 *  inet_pton(AF_INET6, "fe80::dbff:b21b:fe78:20b", &ipv6addr.sin6_addr);
 *  ipv6addr.sin6_scope_id = 1;
 *  @endcode
 *
 *  ## Storing The Server List In A Contiguous Block Of Memory ##
 *
 *  Once the address structures are properly initialized, they need to be
 *  written into a contiguous block of memory.  This can be done copying the
 *  address structures into a byte array:
 *  @code
 *  #define SIZE (sizeof(struct sockaddr_in) + sizeof(struct sockaddr_in6))
 *  unsigned char ntpServers[SIZE];
 *  int currPos = 0;
 *
 *  memcpy((ntpServers + currPos), &ipv4addr, sizeof(struct sockaddr_in));
 *  currPos += sizeof(struct sockaddr_in);
 *
 *  memcpy((ntpServers + currPos), &ipv6addr, sizeof(struct sockaddr_in6));
 *  currPos += sizeof(struct sockaddr_in6);
 *  @endcode
 *
 *  ## Starting The SNTP Client Service ##
 *
 *  The SNTP client should be initialized by calling the SNTP_start()
 *  function.
 *  Pointers to the set and get time functions should be passed as arguments,
 *  as well as for the (optional) user callback function that is used in
 *  conjunction with
 *  SNTP_forceTimeSync().  The pre-initialized list of servers and number of
 *  servers
 *  should be passed next.  Finally, the last argument to pass is the stack
 *  size to be used for the SNTP client Task, if desired.
 *
 *  The following example code passes pointers to the SYS/BIOS Seconds module's
 *  time functions 'Seconds_get' and 'Seconds_set', along with a user defined
 *  hook function. The list of NTP servers, as defined in the previous section
 *  are also passed.  Finally, a value of 0 is passed as
 *  the last argument, which specifies that the default stack size for the SNTP
 *  client Task should be used:
 *  @code
 *  SNTP_start(Seconds_get, Seconds_set, timeUpdateHook, (struct sockaddr *)ntpServers, 2, 0);
 *  @endcode
 *
 *  ## Updating The Server List ##
 *
 *  The list of NTP servers can be updated at a later point in time in the
 *  application.  To update the list of NTP servers, call SNTP_setServers(),
 *  passing the new server
 *  list and the number of servers in the list as arguments.  Note that the
 *  server list is cast to the generic type of 'struct sockaddr *':
 *  @code
 *  SNTP_setServers((struct sockaddr *)ntpServers, 2);
 *  @endcode
 *
 *  ## Stopping The SNTP Client Service ##
 *
 *  To stop the SNTP client, the SNTP_stop() function is called:
 *  @code
 *  SNTP_stop();
 *  @endcode
 *
 *  ## Forcing The SNTP Client To Syncrhonize The Time ##
 *
 *  To force the SNTP client to wake up and synchronize the time with an NTP
 *  server, call SNTP_forceTimeSync():
 *  @code
 *  SNTP_forceTimeSync();
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef SNTP_
#define SNTP_

#include <xdc/std.h>

#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SNTP_start ========
 */
/*!
 *  @brief Initialize and start the SNTP client Task.
 *
 *  Called to create and start SNTP client Task and Semaphores.  User must
 *  pass in pointers to functions for getting and setting the current time and
 *  the list of NTP servers to communicate with.
 *
 *  The SNTP client Task's stacksize is configurable via the stacksize
 *  parameter.
 *
 *  Passing a value of 0 for the stack parameter causes the Task to be created
 *  with a default value.  Passing a value > 0 causes the Task to be created
 *  using that value for the stack size. When a value > 0 is passed, it is the
 *  application developer's responsiblity to choose an appropriate stack size.
 *  If this is not done, stack overflow or other undesirable behavior may occur.
 *
 *  The priority of SNTP_synctime is set to 1 and is not configurable.
 *
 *  @param  get A pointer to a function that returns the number of seconds since
 *              January 1, 1970.
 *
 *  @param  set A pointer to a function that sets the time to the supplied
 *              value.
 *
 *  @param  timeUpdatedHook (optional) A pointer to a function that will be
 *                          called upon successful time synchronization with an
 *                          NTP server after a call to SNTP_forceTimeSync() was
 *                          made.
 *
 *  @param servers A pointer to a contiguous block of memory which contains a
 *                 list of sockaddr_in and/or sockaddr_in6 structures.
 *
 *  @param numservers The number of servers (structs) contained in the server
 *                    list.
 *
 *  @param stacksize Stack size to use for the SNTP client Task. Passing a value
 *                   of 0 allows a default to be used.
 *
 *  @return 0 on failure and 1 on success
 */
extern int SNTP_start(uint32_t (*get)(void), void (*set)(uint32_t newtime),
        void (*timeUpdatedHook)(void *), struct sockaddr *servers,
        unsigned int numservers, size_t stacksize);

/*
 *  ======== SNTP_stop ========
 */
/*! @brief Stops the SNTP client Task and frees resources used by the module */
extern void SNTP_stop(void);

/*
 *  ======== SNTP_setServers ========
 */
/*!
 *  @brief Updates the list of NTP servers to communicate with.
 *
 *  Must call SNTP_start() first.  This function is used to change the list of
 *  NTP servers that were passed as arguments in the initial call to
 *  SNTP_start().
 *
 *  @param servers A pointer to a contiguous block of memory which contains a
 *                 list of sockaddr_in and/or sockaddr_in6 structures.
 *
 *  @param numservers The number of servers (structs) contained in the server
 *                    list.
 */
extern void SNTP_setServers(struct sockaddr *servers, unsigned int numservers);

/*
 *  ======== SNTP_forceTimeSync ========
 */
/*!
 *  @brief Force SNTP client to get the time from a configured NTP server.
 *
 *  Must call SNTP_start() first.  This function is used to unblock the SNTP
 *  client thread in order to initiate communication with one of the configured
 *  NTP servers and update the time.  Upon successful time update, the SNTP
 *  client task will call the user supplied time upate hook function, which was
 *  (optionally) configured in the call to SNTP_start(). Note that this hook
 *  will only be called if SNTP_forceTimeSync() was called to update the time.
 *
 *  Per RFC 4330, this API must not be called more than one time in any given
 *  15 second period of time!
 */
extern void SNTP_forceTimeSync(void);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* SNTP_ */
