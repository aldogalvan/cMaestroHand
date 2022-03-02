#include "esmacat_application.h"

esmacat_application::esmacat_application()
{
    ecat_server = new esmacat_server;           // In the server, all EtherCAT events are happening
    printf ("EsmaCAT server is created!\n");    // YAHO!
}

esmacat_err esmacat_application::assign_esmacat_slave_index(esmacat_analog_input*& slave_in_app, int slave_index){
    if ( ecat_server->ECAT_slave[slave_index]->get_esmacat_product_id() == ESMACAT_ANALOG_INPUT_ID ){   // if the selected physical esmacat slave is really analog input module
        slave_in_app =(esmacat_analog_input*) ecat_server->ECAT_slave[slave_index];
        return NO_ERR;
    }
    else{
        printf("EsmaCAT Product ID: %x \n", ecat_server->ECAT_slave[slave_index]->get_esmacat_product_id() );
        printf("EsmaCAT 16CH ANALOG INPUT ID: %x", ESMACAT_ANALOG_INPUT_ID );
        perror("ERR_ESMACAT_SLAVE_DOES_NOT_MATCH");
        ecat_server->stop_thread();
        return ERR_ESMACAT_SLAVE_DOES_NOT_MATCH;
    }
}

esmacat_err esmacat_application::assign_esmacat_slave_index(esmacat_motor_driver*& slave_in_app, int slave_index){
    if ( ecat_server->ECAT_slave[slave_index]->get_esmacat_product_id() == ESMACAT_MOTOR_DRIVER_ID ){ // if consistent
        slave_in_app =(esmacat_motor_driver*) ecat_server->ECAT_slave[slave_index];
        slave_in_app->set_esmacat_app_one_cycle_time_sec( ESMACAT_ONE_CYCLE_TIME_NANO_SEC * 1e-9  );
        return NO_ERR;
    }
    else{
        printf("EsmaCAT Product ID: %x \n", ecat_server->ECAT_slave[slave_index]->get_esmacat_product_id() );
        printf("EsmaCAT MOTOR DRIVER ID: %x", ESMACAT_MOTOR_DRIVER_ID );
        perror("ERR_ESMACAT_SLAVE_DOES_NOT_MATCH MD");
        ecat_server->stop_thread();
        return ERR_ESMACAT_SLAVE_DOES_NOT_MATCH;
    }
}

esmacat_err esmacat_application::assign_esmacat_slave_index(esmacat_sea_driver*& slave_in_app, int slave_index){
    if ( ecat_server->ECAT_slave[slave_index]->get_esmacat_product_id() == ESMACAT_SEA_DRIVER_ID ){ // if consistent
        slave_in_app =(esmacat_sea_driver*) ecat_server->ECAT_slave[slave_index];
        slave_in_app->set_esmacat_app_one_cycle_time_sec( ESMACAT_ONE_CYCLE_TIME_NANO_SEC * 1e-9  );
        return NO_ERR;
    }
    else{
        perror("ERR_ESMACAT_SLAVE_DOES_NOT_MATCH WITH SEA ");
        ecat_server->stop_thread();
        return ERR_ESMACAT_SLAVE_DOES_NOT_MATCH;
    }
}


esmacat_err esmacat_application::assign_esmacat_slave_index(esmacat_loadcell_interface*& slave_in_app, int slave_index){
    if ( ecat_server->ECAT_slave[slave_index]->get_esmacat_product_id() == ESMACAT_LOADCELL_INTERFACE_ID){ // if consistent
        slave_in_app =(esmacat_loadcell_interface*) ecat_server->ECAT_slave[slave_index];
        return NO_ERR;
    }
    else{
        perror("ERR_ESMACAT_SLAVE_DOES_NOT_MATCH");
        ecat_server->stop_thread();
        return ERR_ESMACAT_SLAVE_DOES_NOT_MATCH;
    }
}



bool esmacat_application::is_esmacat_server_closed(){
    ecat_server->is_ec_closed();
}

void esmacat_application::setup(){
    // do something for initial setup
}

void esmacat_application::set_elapsed_time_ms(double elapsed_time_ms_){
    elapsed_time_ms = elapsed_time_ms_;
}

void esmacat_application::loop(){
    // this is a base function, needs to be overwrode by a child class
}

void esmacat_application::start()
{
    ecat_server->assign_esmacat_application(this);
    ecat_server->StartInternalThread();
}

void esmacat_application::set_ethercat_adapter_name(char* eth_adapter_name){
    ecat_server->set_ethernet_adapter_name_for_esmacat(eth_adapter_name);
}

void esmacat_application::set_ethercat_adapter_name_through_terminal(){
    // declare variables for ethercat
    ec_adaptert * adapter = NULL;
    ec_adaptert * adapter_selected = NULL;
    int adapter_index = 0;
    int selected_adapter_index = 0;
    int i=0;
    printf("---------------------------------------------- \n");
    printf("EsmaCAT Master Software\n");
    printf("---------------------------------------------- \n");
    printf ("List of available Ethernet adapters\n");
    adapter = adapter_selected = ec_find_adapters ();
    while (adapter != NULL)
    {
        printf ("%d: %s\n", adapter_index++, adapter->name);
        adapter = adapter->next;
    }
    printf("---------------------------------------------- \n");
    if (adapter_index == 0){
        printf ("There is no available adapter" );
    }
    else{
        printf ("Please select the adapter for EtherCAT: (0 - %d): ",adapter_index-1 );
        scanf("%d",&selected_adapter_index);
    }
    for (i=0;i<selected_adapter_index;i++) adapter_selected=adapter_selected->next;
    set_ethercat_adapter_name(adapter_selected->name);
}

void esmacat_application::stop(){
    ecat_server->stop_thread();
}
