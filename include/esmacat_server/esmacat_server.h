#ifndef ESMACAT_SERVER_H
#define ESMACAT_SERVER_H
#define EC_TIMEOUTMON 500
#define ESMACAT_ONE_CYCLE_TIME_NANO_SEC 500000

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>
#include "ethercat.h"
#include "esmacat_application.h"
#include "esmacat_slave.h"

// start of RT include
#include <sys/mman.h>
#define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */

#define NSEC_PER_SEC    (1000000000) /* The number of nsecs per sec. */


// end of RT define

class esmacat_application;

class esmacat_server
{
public:
    esmacat_server();         // constructr, initializing all variables
    ~esmacat_server();
    bool StartInternalThread();                         // start the threads for esmacat server, Returns true if the hThread was successfully started, false if there was an error starting the hThread
    void WaitForInternalThreadToExit();     // close the threads. Call this function when you finish the program.
    void esmacat_server_loop();                         // main RT loop
    void ecatcheck();                                   // check the status of EtherCAT
    bool is_ec_closed(){return ec_closed;}
    void assign_esmacat_application(esmacat_application* esmacat_app){ECAT_app=esmacat_app;}
    //    std::vector<esmacat_slave*> get_ECAT_slave_vector(){return ECAT_slave;}
    std::vector<esmacat_slave*> ECAT_slave;             // array of esmacat slaves
    void stop_thread();                                 // stop threads of esmacat server
    void set_ethernet_adapter_name_for_esmacat(char* eth_adapter_name);

private:
    static void * InternalThreadEntry_esmacat_server_loop(void * This) {((esmacat_server *)This)->esmacat_server_loop(); return NULL;}  // this is an intermedaite function to use pthread
    static void * InternalThreadEntry_ecatcheck(void * This) {((esmacat_server *)This)->ecatcheck(); return NULL;}                      // this is an intermedaite function to use pthread

    char IOmap[4096];   // IO map for EtherCAT, used by SOEM
    int expectedWKC;    // part of SOEM
    boolean needlf;     // part of SOEM
    volatile int wkc;   // part of SOEM
    boolean inOP;       // part of SOEM
    uint8 currentgroup; // part of SOEM
    bool ec_closed;     // part of SOEM
    bool stop_thread_loop;  // part of SOEM
    esmacat_application* ECAT_app; // esmacat application will be copied to here and be used in threads
    OSAL_THREAD_HANDLE _thread_esmacat_server_loop;
    OSAL_THREAD_HANDLE _thread_ecatcheck;
    char ifname[100];   // ethernet adapter name
};

#endif // ESMACAT_SERVER_H
