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

#include <sys/mman.h>
#include "ethercat.h"
#include "math.h"
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>

#define EC_TIMEOUTMON 500
#define STATUS_WORD_MASK(x)          (x &= 0x6F)

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
char usdo[128];
char hstr[1024];

void set_output_int16 (uint16_t slave_nb, uint8_t module_index, int16_t value)
{
    uint8_t *data_ptr;

    data_ptr = ec_slave[slave_nb].outputs;
    // Move pointer to correct module inde
    data_ptr += module_index * 2;
    // Read value byte by byte since all targets can't handle misaligned addresses
    *data_ptr++ = (value >> 0) & 0xFF;
    *data_ptr++ = (value >> 8) & 0xFF;
}

void set_output_int32 (uint16_t slave_nb, uint8_t module_index, int32_t value)
{
    set_output_int16(slave_nb, module_index, (int16_t)(value & 0xffff) );
    set_output_int16(slave_nb, module_index+0x02, (int16_t)((value >> 16) & 0xffff));
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

void get_input_int16(uint16_t slave_nb, uint8_t module_index, int16_t *value)
{
    uint8_t *data_ptr;

    data_ptr = ec_slave[slave_nb].inputs;
    /* Move pointer to correct module index*/
    data_ptr += module_index * 2;
    /* Read value byte by byte since all targets can't handle misaligned addresses */
    *value |= ((*data_ptr++) & 0xFF);
    *value |= ((*data_ptr) << 8) & 0xff00;
}


void get_input_int32(uint16_t slave_nb, uint8_t module_index, int32_t *value)
{
    int16_t value_high = 0;
    int16_t value_low = 0;
    
    get_input_int16(slave_nb, module_index, (int16_t*) &value_low);
    get_input_int16(slave_nb, module_index+0x02, (int16_t*) &value_high);
    *value = ((value_high << 16) & 0xffff0000) | (value_low & 0x00ffff);
}

static int set_pdo_write8 (uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
   return wkc;
}

static int set_pdo_write16 (uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
   return wkc;
}

static int set_pdo_write32 (uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,  EC_TIMEOUTRXM);
   return wkc;
}

int ec_config_pdo(uint16 slave)
{
//          SM2 outputs
//             addr b   index: sub bitl data_type    name
//          [0x0000.0] 0x6040:0x00 0x10 UNSIGNED16   Controlword
//          [0x0002.0] 0x607A:0x00 0x20 INTEGER32    Target position
//          [0x0006.0] 0x60B1:0x00 0x20 INTEGER32    Velocity offset
//          [0x000A.0] 0x60B2:0x00 0x10 INTEGER16    Torque offset
//          [0x000C.0] 0x6060:0x00 0x08 INTEGER8     Modes of operation
//          [0x000D.0] 0x0000:0x00 0x08
//          SM3 inputs
//             addr b   index: sub bitl data_type    name
//          [0x000E.0] 0x6041:0x00 0x10 UNSIGNED16   Statusword
//          [0x0010.0] 0x6064:0x00 0x20 INTEGER32    Position actual value
//          [0x0014.0] 0x606C:0x00 0x20 INTEGER32    Velocity actual value
//          [0x0018.0] 0x6077:0x00 0x10 INTEGER16    Torque value
//          [0x001A.0] 0x6061:0x00 0x08 INTEGER8     Modes of operation display
//          [0x001B.0] 0x0000:0x00 0x08
    int wsum = 0;

    wsum += set_pdo_write8 (slave, 0x6060, 0, 10); // Set mode of operation to CST
    uint16 indexI = 0x1607;
    uint16 indexO = 0x1A07;

    wsum += set_pdo_write8 (slave, 0x1C12, 0, 0);
    wsum += set_pdo_write8 (slave, indexI, 0, 0);
    wsum += set_pdo_write32(slave, indexI, 1, 0x607A0020); // Target position
    wsum += set_pdo_write32(slave, indexI, 2, 0x60FF0020); // Velocity offset
    wsum += set_pdo_write32(slave, indexI, 3, 0x60710010); // Target torque
    wsum += set_pdo_write32(slave, indexI, 4, 0x60400010); // Control word
    //wsum += set_pdo_write32(slave, 0x1600, 5, 0x60600008); // mode of operation
    //wsum += set_pdo_write32(slave, 0x1600, 6, 0x00000008); // none
    // wsum += set_pdo_write32(slave, 0x1600, 7, 0x60FE0020); // IO
    //wsum += set_pdo_write32(slave, 0x1600, 4, 0x60B20010); // torque offset
    wsum += set_pdo_write8 (slave, indexI, 0, 4);
    wsum += set_pdo_write16(slave, 0x1C12, 1, indexI);
    wsum += set_pdo_write8 (slave, 0x1C12, 0, 1);


    wsum += set_pdo_write8 (slave, 0x1C13, 0, 0);
    wsum += set_pdo_write8 (slave, indexO, 0, 0);
    wsum += set_pdo_write32(slave, indexO, 1, 0x60640020); // Position actual value
    wsum += set_pdo_write32(slave, indexO, 2, 0x606C0020); // Velocity actual value
    wsum += set_pdo_write32(slave, indexO, 3, 0x60770010); // torque offset
    wsum += set_pdo_write32(slave, indexO, 4, 0x60410010); // Status word
    //wsum += set_pdo_write32(slave, indexO, 5, 0x60610008); // mode of operation display
    //wsum += set_pdo_write32(slave, 0x1A00, 6, 0x00000008); // none
    //wsum += set_pdo_write32(slave, indexO, 6, 0x60FD0020); // IO
    wsum += set_pdo_write8 (slave, indexO, 0, 4);
    wsum += set_pdo_write16(slave, 0x1C13, 1, indexO);
    wsum += set_pdo_write8 (slave, 0x1C13, 0, 1);

    return wsum;
}

boolean ethercat_initial(char *ifname)
{
    if (inOP) {
        return TRUE;
    }

    int i, chk;
    needlf = FALSE;
    inOP = FALSE;
  //  int16_t statusword = 0;

    printf("Starting initial\n");

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

            int slave;
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                ec_config_pdo(slave);
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

                int16_t StateWord = 0;
                int16_t JointCurTor = 0;
                int32_t JointCurPos = 0;
                uint8  Mode = 10;

                // Motor State: reset
                int16_t ControlWord = 128;
                set_output_int16(1, 0x0A >> 1, ControlWord);

                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);

                get_input_int16 (1, 0x0A >> 1, (int16_t *)&StateWord);
                get_input_int32 (1, 0x00 >> 1, &JointCurPos);
                printf("Motor State: %d, Mode: %d, CurPos: %d\n", StateWord, Mode, JointCurPos);

                // Motor State: ready to switch on
                ControlWord = 6;
                set_output_int16(1, 0x0A >> 1, ControlWord);
                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);

                get_input_int16 (1, 0x0A >> 1, &StateWord);
                get_input_int32 (1, 0x00 >> 1, &JointCurPos);
                printf("State: %d, Mode: %d, CurPos: %d\n", StateWord, Mode, JointCurPos);

                // Motor State: switch on
                ControlWord = 7;
                set_output_int16(1, 0x0A >> 1, ControlWord);
                set_output_int16(1, 0x08 >> 1, JointCurTor);
                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);

                get_input_int16 (1, 0x0A >> 1, &StateWord);
                printf("State: %d, Mode: %d, CurPos: %d\n", StateWord, Mode, JointCurPos);

                osal_usleep(10000);
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
    int16_t ControlWord = 128;
    int16_t StateWord = 0;
    set_output_int16(1, 0x0A >> 1, ControlWord);

    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(100000);

    get_input_int16 (1, (0x0E - 0x0E) >> 1, &StateWord);
    printf("Motor State: %d\n", StateWord);

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

        if (inOP) {
            ec_send_processdata();
            // wkc = ec_receive_processdata(EC_TIMEOUTRET);
            osal_usleep(10000);
        }
    }
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
	//RTIME now;
   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
//      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);

      struct timeval t;
      long t_max = 0, t_min  = 1000000, t_sum = 0, t0, t1;

      /* start cyclic part */
      if (ethercat_initial(argv[1])) {
        // Motor State: Enable

        int16_t ControlWord = 15, StateWord = 0;
        int32_t JointCurPos = 0;
        int32_t JointCurVel = 0;
        int16_t JointCurTor = 0;
        
        set_output_int16(1, 0x0A >> 1, ControlWord);
        set_output_int16(1, 0x08 >> 1, 0);
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        
        int16_t i = 0;
        for (i = 0; i < 1000; ++ i) {
			//now = rt_timer_read();
            //t0 = now/1000.0;
              gettimeofday(&t, NULL);
              t0 = t.tv_sec * 1000000.0 + t.tv_usec;

            set_output_int16(1, 0x0A >> 1, ControlWord);
            set_output_int16(1, 0x08 >> 1, i);
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            get_input_int32(1, 0x00 >> 1, &JointCurPos);
            get_input_int32(1, 0x04 >> 1, &JointCurVel);
            get_input_int16(1, 0x08 >> 1, &JointCurTor);
            get_input_int16(1, 0x0A >> 1, &StateWord);

			//now = rt_timer_read();
            //t1 = now/1000.0 - t0;
              gettimeofday(&t, NULL);
              t1 = t.tv_sec * 1000000.0 + t.tv_usec - t0;
            t_sum += t1;
            if (t_max < t1)
                t_max = t1;
            if (t_min > t1)
                t_min = t1; 

            if (i % 100 == 0)
                printf("State: %d, Pos: %d, Vel: %d, Tor: %d\n", StateWord, JointCurPos, JointCurVel, JointCurTor);
            
            osal_usleep(1000);
        }
        
        for (i = 0; i < 1000; ++ i) {
			//now = rt_timer_read();
            //t0 = now/1000.0;
              //gettimeofday(&t, NULL);
              //t0 = t.tv_sec * 1000000.0 + t.tv_usec;


            set_output_int16(1, 0x0A >> 1, ControlWord);
            set_output_int16(1, 0x08 >> 1, 1000-i);
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            get_input_int32(1, 0x00 >> 1, &JointCurPos);
            get_input_int32(1, 0x04 >> 1, &JointCurVel);
            get_input_int16(1, 0x08 >> 1, &JointCurTor);
            get_input_int16(1, 0x0A >> 1, &StateWord);

			//now = rt_timer_read();
            //t1 = now/1000.0 - t0;
              /*gettimeofday(&t, NULL);
              t1 = t.tv_sec * 1000000.0 + t.tv_usec - t0;
            t_sum += t1;
            if (t_max < t1)
                t_max = t1;
            if (t_min > t1)
                t_min = t1; */

            if (i % 100 == 0)
                printf("State: %d, Pos: %d, Vel: %d, Tor: %d\n", StateWord, JointCurPos, JointCurVel, JointCurTor);
            
            osal_usleep(1000);
        }

          //printf("max time %d, min time %d, ave time %d us\n", t_max, t_min, t_sum / 2000);
        set_output_int16(1, 0x00 >> 1, 128);
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
            printf("PDO max time %ld, min time %ld, ave time %ld us\n", t_max, t_min, t_sum / 1000);
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
