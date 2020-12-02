/*
    //double PI = acos(-1.0);
    double p0 = read_PDO_item(slave, 0x6064, 0, 4, "Read Inc Cur position") * 360 / 18432 / 120;     // absolute position
    //double pt = p0;
    //double ff = 10;
    uint8  OpertionMode = 8;
    uint16 ControlWord = 0;
    uint32 JointCmdPos = 0;
    read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
    write_PDO_item(slave, 0x6060, 1, &OpertionMode, "Write Mode of Operation"); // mode of opertation
    read_PDO_item(slave, 0x6061, 0, 1, "Read Mode of operaion");    // mode of operation

    usleep(100000);
    ControlWord = 128;
    write_PDO_item(slave, 0x6040, 2, &ControlWord, "Write Controlwoord");    // controlwords: CONTROL_WORD_RESET
    read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

    usleep(100000);
    ControlWord = 6;
    write_PDO_item(slave, 0x6040, 2, &ControlWord, "Write Controlwoord");      // controlwords: CONTROL_WORD_SHUTDOWN
    read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

    usleep(100000);
    JointCmdPos = (uint32)(p0 * 18432 * 120 / 360);
    write_PDO_item(slave, 0x60b0, 4, &JointCmdPos, "Write Inc Tar Position"); // position offset
    read_PDO_item(slave, 0x60b0, 0, 4, "Read Inc Tar Position");

    usleep(100000);
    ControlWord = 7;
    write_PDO_item(slave, 0x6040, 2, &ControlWord, "Write Controlwoord");      // controlwords: CONTROL_WORD_SWITHON
    read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

    usleep(100000);
    ControlWord = 15;
    write_PDO_item(slave, 0x6040, 2, &ControlWord, "Write Controlwoord");     // controlwords: CONTROL_WORD_ENABLE
    read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
    usleep(1000000);
    read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
    usleep(1000000);
    read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
    usleep(1000000);
    read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
    usleep(1000000);
    read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

    for (int i = 0; i < t * ff; ++ i) {
        read_PDO_item(slave, 0x6041, 0, 2, "Statusword");       // state words
        pt = p0 + (p1 - p0) * (1.0 - cos(2.0 * PI * i / t / ff)) * 0.5;
        JointCmdPos = (uint32)(pt * 18432 * 120 / 360);
        // write_PDO_item(slave, 0x60b0, 4, &JointCmdPos, "Write Inc Tar Position"); // position offset
        //write_PDO_item(slave, 0x607a, 4, &JointCmdPos, "Write Inc Tar Position"); // target position
        usleep(100000);
        read_PDO_item(slave, 0x6064, 0, 4, "Read Inc Cur position");
        read_PDO_item(slave, 0x607a, 0, 4, "Read Inc Tar position");
    }
    */

/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"
#include "math.h"
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

void set_output_int16 (uint16_t slave_nb, uint8_t module_index, uint16_t value)
{
    uint8_t *data_ptr;

    data_ptr = ec_slave[slave_nb].outputs;
    // Move pointer to correct module inde
    data_ptr += module_index * 2;
    // Read value byte by byte since all targets can't handle misaligned addresses
    *data_ptr++ = (value >> 0) & 0xFF;
    *data_ptr++ = (value >> 8) & 0xFF;
}

void set_output_int8 (uint16_t slave_nb, uint8_t module_index, uint8_t value)
{
    uint8_t *data_ptr;

    data_ptr = ec_slave[slave_nb].outputs;
    // Move pointer to correct module inde
    data_ptr += module_index * 2;
    // Read value byte by byte since all targets can't handle misaligned addresses
    *data_ptr++ = value & 0xFF;
}

void get_input_int8(uint16_t slave_nb, uint8_t module_index, uint8_t *value)
{
    uint8_t *data_ptr;

    data_ptr = ec_slave[slave_nb].inputs;
    /* Move pointer to correct module index*/
    data_ptr += module_index * 2;
    /* Read value byte by byte since all targets can't handle misaligned addresses */
    *value = *data_ptr;
}

void get_input_int16(uint16_t slave_nb, uint8_t module_index, uint16_t *value)
{
    uint8_t *data_ptr;

    data_ptr = ec_slave[slave_nb].inputs;
    /* Move pointer to correct module index*/
    data_ptr += module_index * 2;
    /* Read value byte by byte since all targets can't handle misaligned addresses */
    *value |= ((*data_ptr++) & 0xFF);
    *value |= ((*data_ptr) << 8) & 0xff00;
//    unsigned int addr = (unsigned long)(data_ptr - ec_slave[slave_nb].inputs);
//    printf("read addr:%x data:%x\n", addr, *value);
}

void simpletest(char *ifname)
{
    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;
  //  int16_t statusword = 0;
    uint8_t value = 0, state = 0;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname)) {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        if ( ec_config_init(FALSE) > 0 ) {
            printf("%d slaves found and configured.\n",ec_slavecount);
            printf("State: %x\n", ec_slave[0].state);
            ec_config_map(&IOmap);
            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
            ec_slave[0].state = EC_STATE_SAFE_OP;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;
            /* wait for all slaves to reach OP state */
            do
            {
               ec_send_processdata();
               ec_receive_processdata(EC_TIMEOUTRET);
               ec_statecheck(0, EC_STATE_SAFE_OP, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_SAFE_OP));

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 8) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 8) iloop = 8;

            printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;
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
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;
/*
                //double PI = acos(-1.0);
                double p0 = read_PDO_item(slave, 0x6064, 0, 4, "Read Inc Cur position") * 360 / 18432 / 120;     // absolute position
                //double pt = p0;
                //double ff = 10;
                uint8  OpertionMode = 8;
                uint16 ControlWord = 0;
                uint32 JointCmdPos = 0;
                read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
                write_PDO_item(slave, 0x6060, 1, &OpertionMode, "Write Mode of Operation"); // mode of opertation
                read_PDO_item(slave, 0x6061, 0, 1, "Read Mode of operaion");    // mode of operation

                usleep(100000);
                ControlWord = 128;
                write_PDO_item(slave, 0x6040, 2, &ControlWord, "Write Controlwoord");    // controlwords: CONTROL_WORD_RESET
                read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

                usleep(100000);
                ControlWord = 6;
                write_PDO_item(slave, 0x6040, 2, &ControlWord, "Write Controlwoord");      // controlwords: CONTROL_WORD_SHUTDOWN
                read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

                usleep(100000);
                JointCmdPos = (uint32)(p0 * 18432 * 120 / 360);
                write_PDO_item(slave, 0x60b0, 4, &JointCmdPos, "Write Inc Tar Position"); // position offset
                read_PDO_item(slave, 0x60b0, 0, 4, "Read Inc Tar Position");

                usleep(100000);
                ControlWord = 7;
                write_PDO_item(slave, 0x6040, 2, &ControlWord, "Write Controlwoord");      // controlwords: CONTROL_WORD_SWITHON
                read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

                usleep(100000);
                ControlWord = 15;
                write_PDO_item(slave, 0x6040, 2, &ControlWord, "Write Controlwoord");     // controlwords: CONTROL_WORD_ENABLE
                read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
                usleep(1000000);
                read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
                usleep(1000000);
                read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
                usleep(1000000);
                read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
                usleep(1000000);
                read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
*/
                for (value = 0; value < 255; ++ value) {
                    set_output_int8(1, (0x11 - 0x09) >> 1, value);
                    set_output_int8(2, (0x11 - 0x09) >> 1, ~value);

                    get_input_int8(1, 0x08 >> 1, &state);

                    printf("button state: %x\n", state);

                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    osal_usleep(5000);
                }

                // write_PDO_item
                /*
                write_PDO_item(slave, 0x6040, 0, 4, 0) // controlwords
                write_PDO_item(slave, 0x6060, 0, 4, 0) // mode of opertation
                write_PDO_item(slave, 0x60b0, 0, 4, 0) // position offset
                write_PDO_item(slave, 0x60b1, 0, 4, 0) // velocity offset
                write_PDO_item(slave, 0x60b2, 0, 4, 0) // torque offset
                 * */

            }else {
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
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

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
            if(!ec_group[currentgroup].docheckstate) {
               printf("Yes: all slaves resumed OPERATIONAL.\n");
            }
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
//      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}
