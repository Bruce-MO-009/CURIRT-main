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
char usdo[128];
char hstr[1024];

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
/*
static int set_pdo_write8 (uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
   return wkc;
}*/

boolean ethercat_initial(char *ifname)
{
    if (inOP) {
        return TRUE;
    }

    int i, chk;
    needlf = FALSE;
    inOP = FALSE;

    printf("Starting initiakl\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname)) {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */

        if ( ec_config_init(FALSE) > 0 ) {

            printf("%d slaves found and configured[%d].\n", ec_slavecount, ec_slave[0].state);

            if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4) == EC_STATE_PRE_OP)
            {
                printf("All Slaves in PRE_OP, and configure PDOs\n");
            }

            ec_config_map(&IOmap);
            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
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
                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);
                inOP = TRUE;
                return TRUE;
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

                return FALSE;
            }
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();

        return FALSE;
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
        return FALSE;
    }
}

void ethercat_close()
{
    printf("\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    /* request INIT state for all slaves */
    ec_writestate(0);

    printf("End simple test, close socket\n");
    /* stop SOEM, close socket */
    ec_close();
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
/*
        if (inOP) {
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            osal_usleep(1000);
        }
        * */
    }
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);

      /* start cyclic part */
      struct timeval t;
      long t_max = 0, t_min  = 1000000, t_sum = 0, t0, t1;
      if (ethercat_initial(argv[1])) {
          int i = 0, cnt = 512;
          for (i = 0; i < cnt; ++ i) {
              gettimeofday(&t, NULL);
              t0 = t.tv_sec * 1000000 + t.tv_usec;
              
              set_output_int8(1, 0x00, (char)(i % 256));
              ec_send_processdata();
              wkc = ec_receive_processdata(EC_TIMEOUTRET);
              
              gettimeofday(&t, NULL);
              t1 = t.tv_sec * 1000000 + t.tv_usec - t0;
              t_sum += t1;
              if (t_max < t1)
                  t_max = t1;
              if (t_min > t1)
                  t_min = t1; 
              osal_usleep(5000);
          }
          printf("max time %ld, min time %ld, ave time %ld us\n", t_max, t_min, t_sum / cnt);

          ethercat_close();
      }
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}
