#include "esmacat_server.h"

// start of RT function
void stack_prefault(void) {
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
    return;
}
// end of RT function

template<typename FwdIterator>
void deleter(FwdIterator from, FwdIterator to)
{
    while ( from != to )
    {
        delete *from;
        from++;
    }
}

esmacat_server::~esmacat_server(){
    WaitForInternalThreadToExit();  // join the threads.
}

void esmacat_server::stop_thread(){
    stop_thread_loop = 1;
    WaitForInternalThreadToExit();
}

esmacat_server::esmacat_server(){
    currentgroup = 0;
    ec_closed = 0;
    stop_thread_loop = 0;
}

bool esmacat_server::StartInternalThread(){
    int ret1, ret2;
    ret1 = osal_thread_create(&_thread_ecatcheck, 128000, (void*)&esmacat_server::InternalThreadEntry_ecatcheck, this );        // create a hThread for ecatcheck. This is a cross-platform hThread-creation.
    ret2 = osal_thread_create_rt(&_thread_esmacat_server_loop, 128000, (void*)&InternalThreadEntry_esmacat_server_loop, this);  // create a hThread for esmacat_server. This is a cross-platform hThread-creation.
    if (ret1 ==0 && ret2 == 0)    return 0; // if successful, return 0
    else return 1;                          // else return 1
}

void esmacat_server::ecatcheck(){ // continuously check the ethercat status
    int slave;
    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            if(!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
        if (stop_thread_loop == TRUE) break;
    }
}

void esmacat_server::esmacat_server_loop(){
//    strcpy(ifname,"enp2s0");
//    *ifname = "enp2s0";
    // start of rt part
    struct timespec t;
    struct timespec t_prev;
    struct timespec t_now;
    struct sched_param param;
    int interval = ESMACAT_ONE_CYCLE_TIME_NANO_SEC;
    double time = 0;
    long d;
    bool enable_flag = true;
    /* Declare ourself as a real time task */
    param.sched_priority = MY_PRIORITY;
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }
    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        perror("mlockall failed");
    }
    /* Pre-fault our stack */
    stack_prefault();

    clock_gettime(CLOCK_MONOTONIC ,&t);
    /* start after one second */
    clock_gettime(CLOCK_MONOTONIC ,&t_prev);

    t.tv_sec++;
    // end of rt part

    int i, j, oloop, iloop, chk,cnt;
    double time_elapsed = 0;
    i = 0;
    needlf = FALSE;
    inOP = FALSE;
    int sum_of_system_parameter_buffer = 0;



    printf("\nStarting esmacat Master Server\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);

            ec_config_map(&IOmap);
            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            /* read indevidual slave state and store in ec_slave[] */
            ec_readstate();

            // read slave information and, recognize the slave
            for(cnt = 1; cnt <= ec_slavecount ; cnt++)
            {
                printf("\nSlave Index: %d\n", cnt-1);
                printf("Manufacturer code: %8.8x Name: %s Product ID: %8.8x,  Rev: %8.8x iByte: %d oByte: %d\n", ec_slave[cnt].eep_man, ec_slave[cnt].name, ec_slave[cnt].eep_id, ec_slave[cnt].eep_rev, ec_slave[cnt].Ibytes,ec_slave[cnt].Obytes);

                switch (ec_slave[cnt].eep_id) {
//                case ESMACAT_MULITFUCTION_ID:
//                    ECAT_slave.push_back( new esmacat_Multifunction );
//                    break;
                case ESMACAT_MOTOR_DRIVER_ID:
                    ECAT_slave.push_back( new esmacat_motor_driver );
                    break;
                case ESMACAT_LOADCELL_INTERFACE_ID:
                    ECAT_slave.push_back( new esmacat_loadcell_interface );
                    break;
                case ESMACAT_ANALOG_INPUT_ID:
                    ECAT_slave.push_back( new esmacat_analog_input);
                    break;
                case ESMACAT_SEA_DRIVER_ID:
                    ECAT_slave.push_back( new esmacat_sea_driver);
                    break;
                default:
                    ECAT_slave.push_back( new esmacat_slave );
                }
                ECAT_slave.back()->set_slave_index(cnt-1);
            }
//            ECAT_app->init(&ECAT_slave);

            ECAT_app->connect();
            ECAT_app->setup();

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
//            printf("Calculated workcounter %d\n", expectedWKC);
//            printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);
//            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
//            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */

            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("\nOperational state reached for all slaves.\n");
                inOP = TRUE;
                /* cyclic loop */
                while(1){
//                    // infinite loop
                    if (i++ == INT32_MAX) i = 0;

                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if(wkc >= expectedWKC)
                    {
//                        printf("Processdata cycle %4d, WKC %d , O:", i, wkc);
                        sum_of_system_parameter_buffer = 0;
                        for(cnt = 1; cnt <= ec_slavecount  ; cnt++)
                        {
                            oloop = ec_slave[cnt].Obytes;
                            if ((oloop == 0) && (ec_slave[cnt].Obits > 0)) oloop = 1;
                            iloop = ec_slave[cnt].Ibytes;
                            if ((iloop == 0) && (ec_slave[cnt].Ibits > 0)) iloop = 1;
                            ECAT_slave[cnt-1]->ecat_data_process( (uint8_t*)&(ec_slave[cnt].outputs[0]),oloop,(uint8_t*)&(ec_slave[cnt].inputs[0]),iloop);
                            sum_of_system_parameter_buffer +=  ECAT_slave[cnt-1]->flush_one_set_of_system_parameters();
                        }
                        if (sum_of_system_parameter_buffer == 0)    {
                            ECAT_app->set_elapsed_time_ms(time_elapsed);
                            ECAT_app->loop();
                        }

                        else {
                            // do nothing.
                        }
//                        printf(" O:");
//                        for(j = 0 ; j < oloop; j++)
//                        {
//                            printf(" %2.2x", *(ec_slave[0].outputs + j));
//                        }
//                        printf("\n");

//                        printf(" I:");
//                        for(j = 0 ; j < iloop; j++)
//                        {
//                            printf(" %2.2x", *(ec_slave[0].inputs + j));
//                        }
//                        printf("\n");

//                        printf(" T:%"PRId64"\r",ec_DCtime);
                        needlf = TRUE;
                    }
//                    osal_usleep(5000);
                    /* wait until next shot */
                    // start of rt sleep
                    /* calculate next shot */
                    t.tv_nsec += interval;

                    while (t.tv_nsec >= NSEC_PER_SEC) {
                        t.tv_nsec -= NSEC_PER_SEC;
                        t.tv_sec++;
                    }

                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

                    clock_gettime(CLOCK_MONOTONIC ,&t_now);
                    d = t_now.tv_nsec - t_prev.tv_nsec;
                    if( d < 0) d = d+NSEC_PER_SEC;
                    t_prev.tv_nsec = t_now.tv_nsec;

                    time_elapsed += ((double)d/1000000);
                    // end f rt sleep

                    if (stop_thread_loop == TRUE) break;
                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("EsmaCAT Server, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
    deleter(ECAT_slave.begin(),ECAT_slave.end());
    ec_closed=1;
}

void esmacat_server::set_ethernet_adapter_name_for_esmacat(char* eth_adapter_name){
    strcpy(ifname, eth_adapter_name);
}

void esmacat_server::WaitForInternalThreadToExit(){
}
