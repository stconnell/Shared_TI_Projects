/*
 * Copyright (c) 2014-2015, Texas Instruments Incorporated
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
 *  ======== sntp.c ========
 *  Contains the SNTP client 'daemon' (a Task that runs every 30 minutes to
 *  sync our time with time value received from an NTP server).
 */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <stdint.h>
#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

#include <sys/socket.h>
#include <ti/ndk/nettools/sntp/sntp.h>

/*
 *  Time Base Conversion Macros
 *
 *  The NTP timebase is 00:00 Jan 1 1900.  The local
 *  time base is 00:00 Jan 1 1970.  Convert between
 *  these two by added or substracting 70 years
 *  worth of time.  Note that 17 of these years were
 *  leap years.
 */
#define TIME_BASEDIFF        ((((unsigned int)70 * 365 + 17) * 24 * 3600))
#define TIME_NTP_TO_LOCAL(t) ((t) - TIME_BASEDIFF)
#define TIME_LOCAL_TO_NTP(t) ((t) + TIME_BASEDIFF)

/* Time to wait after receiving a valid server update (milliseconds) */
#define SNTP_WAIT_TIME 1800000

/* Time to wait for reply from server (seconds) */
#define SNTP_REPLY_WAIT_TIME 5

/* Time to wait before retrying if socket operations fail (milliseconds) */
#define SNTP_SOCKFAIL_WAIT_TIME 30000

/* Max number of Kiss o' Death (KOD) packets to recv before switching server */
#define SNTP_KOD_MAX 4

/* Time to back off (wait) after receiving a KOD packet (milliseconds) */
#define SNTP_KOD_BACKOFF_TIME 15000

/* KOD error code: rate exceeded, server requesting NTP client to back off */
#define SNTP_KOD_RATE_STR "RATE"
#define SNTP_KOD_RATE_CODE 3

/* KOD error code: access denied, server requests client to end all comm */
#define SNTP_KOD_DENY_STR "DENY"
#define SNTP_KOD_DENY_CODE 2

/* KOD error code: access denied, server requests client to end all comm */
#define SNTP_KOD_RSTR_STR "RSTR"
#define SNTP_KOD_RSTR_CODE 1

/* Size of KOD error codes */
#define SNTP_KOD_ERROR_CODE_SIZE 4

/* Event ID used to cause the SNTP daemon to wake up and exit */
#define SNTP_EVENT_STOP Event_Id_00

/* Event ID used to cause the SNTP daemon to wake up and update the time */
#define SNTP_EVENT_FORCEUPDATE Event_Id_01

/* Use NTP version 4 */
#define SNTP_VERSION 4

/* Flag value for unsync'ed leap indicator field, signifying server error */
#define SNTP_NOSYNC 3

/* NTP mode defined in RFC 4330 */
#define SNTP_MODE_CLIENT 3

/* Well known SNTP server port */
#define SNTP_SERVER_PORT 123

/* syncTime Task defaults */
#define SNTP_SYNCTIME_STACKSIZE 1280

/* SNTP Header (as specified in RFC 4330) */
typedef struct _SNTP_Header {
    /*
     *  'flags' stores three values:
     *
     *    - 2 bit Leap Indicator (LI)
     *    - 3 bit Version Number (VN)
     *    - 3 bit Mode.
     */
    unsigned char flags;
    unsigned char stratum;
    unsigned char poll;
    signed char   precision;
    int           rootDelay;
    unsigned int  rootDispersion;
    unsigned int  referenceID;

    /* NTP time stamps */
    unsigned int referenceTS[2];
    unsigned int originateTS[2];
    unsigned int receiveTS[2];
    unsigned int transmitTS[2];
} _SNTP_Header;

/*
 *  Define types and global variables used for calling set and get time
 *  functions.
 */
typedef uint32_t (*GetTimeFxn)(void);
typedef void (*SetTimeFxn)(uint32_t newtime);
typedef void (*SntpCbFxn)(void *);

static GetTimeFxn gettime = NULL;
static SetTimeFxn settime = NULL;
static SntpCbFxn timeUpdCb = NULL;

/* The list of NTP servers to communicate with */
static struct sockaddr *serverList = NULL;

/* The number of NTP servers in the serverList */
static unsigned int numServers = 0;

/* The byte offset of the current server in the serverList */
static int currSrvBytePos = 0;

/* The current server number (used to know when last server in list reached) */
static int currSrvNum = 0;

/* Flag used to avoid re-initialization */
static int initialized = 0;

/* Semaphore used for global variable protection */
static Semaphore_Handle sntpMutex = NULL;

/* Event used to signal syncTime Task to exit or force a time update */
static Event_Handle sntpStopOrUpdate = NULL;

/* Semaphore used to signal SNTP_stop that syncTime has completed */
static Semaphore_Handle sntpSyncDone = NULL;

/* Handle to the syncTime Task */
static Task_Handle sntpSyncHandle = NULL;

/* Function prototypes */
static void changeServer(unsigned char family);
static int hasKissCode(char *str);
static int socketSetup(int *s, struct sockaddr *cs, struct timeval *to);
static void syncTime(unsigned int arg0, unsigned int arg1);

/*
 *  ======== changeServer ========
 *
 *  Utility function to move the current server to point to the next one in the
 *  list.  If we're currently at the last server in list, then reset back to
 *  the beginning of serverList.
 *
 *  The 'family' parameter is the family type of the current server (AF_INET or
 *  AF_INET6).
 *
 *  Must be called with appropriate Semaphore protection (using sntpMutex)
 */
static void changeServer(unsigned char family)
{
    if (currSrvNum == (numServers - 1)) {
        /*
         *  Edge case: current server is last server in serverList, reset
         *  currSrvNum and currSrvBytePos back to first server in list.
         */
        currSrvNum = 0;
        currSrvBytePos = 0;
    }
    else {
        /* Update server number to the next server in our list */
        currSrvNum++;

        /*
         *  Update byte offset to point to next server. Number of bytes to
         *  move depends on the family type of the current server.
         */
        if (family == AF_INET) {
            currSrvBytePos += sizeof(struct sockaddr_in);
        }
#ifdef _INCLUDE_IPv6_CODE
        else if (family == AF_INET6) {
            currSrvBytePos += sizeof(struct sockaddr_in6);
        }
#endif
    }
}

/*
 *  ======== hasKissCode ========
 *
 *  Utility function to check if a string contains a Kiss O' Death code.
 *
 *  Returns:
 *      SNTP_KOD_RATE_CODE - str contains "RATE" KOD code
 *
 *      SNTP_KOD_DENY_CODE - str contains "DENY" KOD code
 *
 *      SNTP_KOD_RSTR_CODE - str contains "RSTR" KOD code
 *
 *      0 - str does not contain any of the above KOD codes
 */
static int hasKissCode(char *str)
{
    if (strncmp((char *)SNTP_KOD_RATE_STR, str, SNTP_KOD_ERROR_CODE_SIZE)
            == 0) {

        return (SNTP_KOD_RATE_CODE);
    }
    else if (strncmp((char *)SNTP_KOD_DENY_STR, str, SNTP_KOD_ERROR_CODE_SIZE)
            == 0) {

        return (SNTP_KOD_DENY_CODE);
    }
    else if (strncmp((char *)SNTP_KOD_RSTR_STR, str, SNTP_KOD_ERROR_CODE_SIZE)
            == 0) {

        return (SNTP_KOD_RSTR_CODE);
    }
    else {
        return (0);
    }
}

/*
 *  ======== socketSetup ========
 *
 *  Utility function to create and connect the SNTP client socket a socket.
 *  This socket can then be used to generically call send() and recv() for
 *  either IPv4 or IPv6.
 *
 *  Returns 1 on success, 0 on failure.
 */
static int socketSetup(int *s, struct sockaddr *cs, struct timeval *to)
{
    int status;

    /* If socket already exists close it */
    if (*s != -1) {
        close(*s);
    }

    /* Create a UDP socket to communicate with NTP server */
    *s = socket(cs->sa_family, SOCK_DGRAM, IPPROTO_UDP);
    if (*s == -1) {
        return (0);
    }

    /* Get size of the current server struct (re-use variable to save space) */
    if (cs->sa_family == AF_INET) {
        status = sizeof(struct sockaddr_in);
    }
#ifdef _INCLUDE_IPv6_CODE
    else if (cs->sa_family == AF_INET6) {
        status = sizeof(struct sockaddr_in6);
    }
#endif
    else {
        return (0);
    }

    /*
     *  Connect our UDP socket. We only want to recv replies from the NTP
     *  server on this socket:
     */
    status = connect(*s, cs, status);
    if (status == -1) {
        return (0);
    }

    /* Set a timeout for server response */
    to->tv_sec  = SNTP_REPLY_WAIT_TIME;
    to->tv_usec = 0;
    if (setsockopt(*s, SOL_SOCKET, SO_RCVTIMEO, to, sizeof(*to)) < 0) {
        return (0);
    }

    /* Socket creation success */
    return (1);
}

/*
 *  ======== SNTP_start ========
 */
int SNTP_start(uint32_t (*get)(void), void (*set)(uint32_t newtime),
        void (*timeUpdatedHook)(void *), struct sockaddr *servers,
        unsigned int numservers, size_t stacksize)
{
    Semaphore_Params semParams;
    Task_Params taskParams;

    /* Don't re-initialize before SNTP_stop is called */
    if (initialized) {
        DbgPrintf(DBG_INFO, "SNTP_start: Error: must call SNTP_stop first.\n");
        goto EXIT_FAIL;
    }

    /* Validate parameters */
    if (!get || !set || !servers || numservers == 0) {
        DbgPrintf(DBG_INFO, "SNTP_start: Error: invalid parameters passed\n");
        goto EXIT_FAIL;
    }

    /* Assign our time getter and setter to functions passed by user */
    gettime = get;
    settime = set;

    /* Store time update callback function, if the user specified one */
    if (timeUpdatedHook) {
        timeUpdCb = timeUpdatedHook;
    }

    /* Semaphore that protects global variable access */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    sntpMutex = Semaphore_create(1, &semParams, NULL);
    if (!sntpMutex) {
        DbgPrintf(DBG_INFO,
                "SNTP_start: Error: Failed to create sntpMutex Semaphore\n");
        goto EXIT_FAIL;
    }

    /* Event used to signal syncTime Task to exit or force a time update */
    sntpStopOrUpdate = Event_create(NULL, NULL);
    if (!sntpStopOrUpdate) {
        DbgPrintf(DBG_INFO,
                "SNTP_start: Error: Failed to create sntpStopOrUpdate Event\n");
        goto EXIT_FAIL;
    }

    /* Semaphore used to signal SNTP_stop that syncTime has completed */
    sntpSyncDone = Semaphore_create(0, &semParams, NULL);
    if (!sntpSyncDone) {
        DbgPrintf(DBG_INFO,
                "SNTP_start: Error: Failed to create sntpSyncDone Semaphore\n");
        goto EXIT_FAIL;
    }

    /*
     *  Create SNTP client daemon
     *
     *  This task will communicate with an SNTP server and sync the system
     *  time to the time received by the server (which is accurately
     *  maintained).
     */
    Task_Params_init(&taskParams);
    taskParams.stackSize =
            (stacksize == 0) ? SNTP_SYNCTIME_STACKSIZE : stacksize;
    taskParams.priority = 1;
    sntpSyncHandle =
            Task_create((Task_FuncPtr)syncTime, &taskParams, NULL);
    if (!sntpSyncHandle) {
        DbgPrintf(DBG_INFO, "SNTP_start: Error: Failed to create synctime Task\n");
        goto EXIT_FAIL;
    }

    /* Set flag indicating that initialization has completed */
    initialized = 1;

    /* call set servers here */
    SNTP_setServers(servers, numservers);

    /* Return success */
    return (1);

EXIT_FAIL:
    /* If any *_create() fails, clean up any successful previous allocations */
    if (sntpMutex) {
        Semaphore_delete(&sntpMutex);
    }
    if (sntpStopOrUpdate) {
        Event_delete(&sntpStopOrUpdate);
    }
    if (sntpSyncDone) {
        Semaphore_delete(&sntpSyncDone);
    }

    /* Return failure */
    return (0);
}

/*
 *  ======== SNTP_stop ========
 */
void SNTP_stop(void)
{
    /* Don't free anything unless we've already initialized! */
    if (!initialized) {
        return;
    }

    /*
     *  Signal syncTime Task to exit
     *
     *  Posting the sntpStopOrUpdate Event (with SNTP_EVENT_STOP) will cause the
     *  Task to break out of its while loop and exit. The Task will then be
     *  cleaned up automatically in the BIOS idle loop (we know this will
     *  happen because NDK requires that Task.deleteTerminatedTasks == TRUE)
     *
     *  Before exiting, synctime will post the sntpSyncDone Semaphore to let
     *  this function know that it has completed and to continue clean up.
     */
    Event_post(sntpStopOrUpdate, SNTP_EVENT_STOP);

    /* Raise the priority of syncTime to allow it to exit */
    Task_setPri(sntpSyncHandle, Task_getPri(Task_self()));

    /* Wait for syncTime to complete/exit */
    Semaphore_pend(sntpSyncDone, BIOS_WAIT_FOREVER);

    /* Delete dynamically allocated objects */
    Semaphore_delete(&sntpSyncDone);
    Event_delete(&sntpStopOrUpdate);
    Semaphore_delete(&sntpMutex);

    /* Reset all global variables */
    sntpSyncDone = NULL;
    sntpStopOrUpdate = NULL;
    sntpMutex = NULL;
    sntpSyncHandle = NULL;
    gettime = NULL;
    settime = NULL;
    timeUpdCb = NULL;
    serverList = NULL;
    numServers = 0;
    currSrvBytePos = 0;
    currSrvNum = 0;

    /* Set flag indicating that de-initialization has completed */
    initialized = 0;
}

/*
 *  ======== SNTP_setServers ========
 */
void SNTP_setServers(struct sockaddr *servers, unsigned int numservers)
{
    /* check for invalid args and ensure sntpMutex has been created */
    if ((!servers) || (numservers == 0) || !initialized) {
        return;
    }

    /* Protect against access by synctime Task */
    Semaphore_pend(sntpMutex, BIOS_WAIT_FOREVER);

    serverList = servers;
    numServers = numservers;

    /* Reset current server tracking variables to beginning of serverList */
    currSrvBytePos = 0;
    currSrvNum = 0;

    Semaphore_post(sntpMutex);
}

/*
 *  ======== SNTP_forceTimeSync ========
 */
void SNTP_forceTimeSync(void)
{
    unsigned int key;

    if (!initialized) {
        return;
    }

    /*
     *  Disable tasks to ensure our event is valid before posting it (It's
     *  possible to be interrupted here and for SNTP_stop() to delete the
     *  event before the program returns to this point and tries posting it).
     */
    key = Task_disable();
        /* Unblock the SNTP daemon to allow communication with NTP server */
        if (sntpStopOrUpdate) {
            Event_post(sntpStopOrUpdate, SNTP_EVENT_FORCEUPDATE);
        }
    Task_restore(key);
}

/*
 *  ======== syncTime ========
 *
 *  Task that runs every 30 minutes to synchronize the local time
 *  with the actual time received from the NTP servers.
 */
static void syncTime(unsigned int arg0, unsigned int arg1)
{
    int sntpSocket = -1;
    struct sockaddr *currServer;
    struct timeval timeout;
    _SNTP_Header sntpPkt;
    fd_set readfds;
    int active = 1;
    int numBytes = 0;
    int numKod = 0;
    int kodBackOffTime = 0;
    int forceTimeSync = 0;
    unsigned int events = 0;
    int status = 0;
    int i = 0;

    /* Initial wait time before sending request to server */
    unsigned int waitTime = 0;

    while (active) {
        /* Sleep here in between server requests */
        if ((events = Event_pend(sntpStopOrUpdate, Event_Id_NONE,
                SNTP_EVENT_STOP + SNTP_EVENT_FORCEUPDATE, waitTime))) {

            if (events & SNTP_EVENT_STOP) {
                /*
                 *  If SNTP_stop() is called, exit out of the loop. (Note that
                 *  this case takes precedence if both events were posted;
                 *  i.e. stop "wins").
                 */
                active = 0;
                goto CONTINUE;
            }
            else if (events & SNTP_EVENT_FORCEUPDATE) {
                /* If SNTP_forceTimeSync() is called, sync with NTP server */
                forceTimeSync = 1;
            }
        }

        /*
         *  Send our request to the current NTP server.  If our current server
         *  responds, then exit this loop, update the time and go back to
         *  sleep for SNTP_WAIT_TIME.
         *
         *  If our current server doesn't respond (e.g. server went down, no
         *  route to server, etc.) then try the next one.
         *
         *  Keep trying until one responds or until we've tried all of the
         *  servers in the serverList. (We only want to try each server once
         *  per wake up cycle).
         *
         *  If we've tried all servers in our list, and did not receive a
         *  response from any of them (e.g. all servers are down!), then we
         *  sleep for SNTP_WAIT_TIME and try the process again upon waking up.
         *
         *  Entire for loop below is guarded by our Semaphore.  We can't allow
         *  changes to the server list while we are actively trying to
         *  communicate with an NTP server(s).
         */

        Semaphore_pend(sntpMutex, BIOS_WAIT_FOREVER);
        for (i = 0; i < numServers; i++) {
            /* Set/update the current server */
            currServer = (struct sockaddr *)((unsigned char *)serverList +
                    currSrvBytePos);

            /* Skip invalid servers (it's invalid if it has a kiss code) */
            if (hasKissCode(currServer->sa_data)) {
                changeServer(currServer->sa_family);
                continue;
            }

            /*
             *  (Re)create socket using family that matches current server.
             *  If the current server's family is IPv6, then this will create
             *  an IPv6 socket.  If family is IPv4, it creates an IPv4 socket.
             */
            if (!socketSetup(&sntpSocket, currServer, &timeout)) {
                /*
                 *  Most likely, stack not up yet or not enough memory.
                 *  Wait and try again.
                 */
                waitTime = SNTP_SOCKFAIL_WAIT_TIME;
                DbgPrintf(DBG_INFO,
                   "syncTime: Error: socket create/init failed (%d)\n",
                    errno);
                goto CONTINUE;
            }

            /* Initialize the SNTP packet, setting version and mode = client */
            memset(&sntpPkt, 0, sizeof(_SNTP_Header));
            sntpPkt.flags = SNTP_VERSION << 3;
            sntpPkt.flags |= SNTP_MODE_CLIENT;

            /* Set packet's transmit time to the current time on our clock */
            sntpPkt.transmitTS[0] = htonl(TIME_LOCAL_TO_NTP((*gettime)()));

            /* Send out our SNTP request to the current server */
            numBytes = send(sntpSocket, (void *)&sntpPkt,
                    sizeof(_SNTP_Header), 0);

            if (numBytes < 0) {
                DbgPrintf(DBG_INFO, "syncTime: Error: failed to send req pkt (%d)\n",
                        errno);
            }

            /* Wait for the reply */
            FD_ZERO(&readfds);
            FD_SET(sntpSocket, &readfds);
            if ((status = select(0, &readfds, NULL, NULL, &timeout)) > 0 ) {
                /*
                 *  We got a response from the current server. Retrieve the NTP
                 *  packet from the socket and update our time.
                 */
                memset(&sntpPkt, 0, sizeof(_SNTP_Header));

                numBytes = recv(sntpSocket, &sntpPkt, sizeof(_SNTP_Header),
                        MSG_WAITALL);

                if ((numBytes < 0) || (numBytes != sizeof(_SNTP_Header))) {
                    DbgPrintf(DBG_INFO, "syncTime: Error: recvfrom failed (%d)\n",
                            (numBytes < 0) ? errno : numBytes);

                    /* Try the next server in our list */
                    changeServer(currServer->sa_family);
                    continue;
                }

                /* Check for errors in server response */
                if (sntpPkt.stratum == 0) {

                    /* Per RFC5905, we MUST handle Kiss O' Death packet */
                    if ((sntpPkt.flags >> 6) == SNTP_NOSYNC) {

                        /* KOD recv'd. Inspect kiss code & handle accordingly */
                        status = hasKissCode((char *)&sntpPkt.referenceID);

                        if (status == SNTP_KOD_RATE_CODE) {
                            /* Server requests that we reduce our send rate */
                            if (numKod++ < SNTP_KOD_MAX) {
                                /* Back off our wait time and retry server */
                                kodBackOffTime += SNTP_KOD_BACKOFF_TIME;
                                waitTime = kodBackOffTime;
                                goto CONTINUE;
                            }
                            else {
                                /* Too many KOD packets. Move to next server */
                                numKod = 0;
                                kodBackOffTime = 0;
                                waitTime = 0;
                                changeServer(currServer->sa_family);
                                continue;
                            }
                        }
                        /* Check for fatal kiss codes */
                        else if (status == SNTP_KOD_DENY_CODE ||
                                status == SNTP_KOD_RSTR_CODE) {
                            /*
                             *  Server requests we end all communication. Mark
                             *  current server as invalid (use kiss code to mark
                             *  it)
                             */
                            memcpy(&currServer->sa_data, &sntpPkt.referenceID,
                                    SNTP_KOD_ERROR_CODE_SIZE);
                            changeServer(currServer->sa_family);
                            continue;
                        }
                        /* Per RFC5905, other kiss codes are ignored */
                    }
                    else {
                        /*
                         *  A server response with stratum == 0, with no kiss
                         *  code, is a fatal error. Mark server as invalid
                         */
                        memcpy(&currServer->sa_data,
                                (char *)SNTP_KOD_DENY_STR,
                                SNTP_KOD_ERROR_CODE_SIZE);
                    }
                }

                /* Use server's transmit time to update our clock */
                (*settime)(
                        TIME_NTP_TO_LOCAL(ntohl(sntpPkt.transmitTS[0])));

                /* If time update was forced, notify of success */
                if (forceTimeSync && timeUpdCb) {
                    (*timeUpdCb)(NULL);
                    forceTimeSync = 0;
                }

                /* Reset KOD count and time once a valid response is received */
                numKod = 0;
                kodBackOffTime = 0;

                /*
                 *  Successful update from our current server, break out and
                 *  sleep for SNTP_WAIT_TIME before updating the time again.
                 */
                break;
            }
            else {
                /* Our current server didn't respond */
                if (status < 0) {
                    DbgPrintf(DBG_INFO,
                        "syncTime: Error waiting for server reply (%d)\n",
                        errno);
                }

                /* Try the next server in our list */
                changeServer(currServer->sa_family);
            }
        }
        waitTime = SNTP_WAIT_TIME;
CONTINUE:
        Semaphore_post(sntpMutex);

        /* Clean up and wait until it's time to sync with server again */
        if (sntpSocket != -1) {
            close(sntpSocket);
        }
    }

    /* Signal SNTP_stop that synctime Task has completed and exit */
    Semaphore_post(sntpSyncDone);
}
