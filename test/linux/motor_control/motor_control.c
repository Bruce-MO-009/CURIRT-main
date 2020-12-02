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

void set_output_int8(uint16_t slave_nb, uint8_t module_index, uint8_t value)
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

    wsum += set_pdo_write8 (slave, 0x6060, 0, 8); // Set mode of operation to CSP

    wsum += set_pdo_write8 (slave, 0x1C12, 0, 0);
    wsum += set_pdo_write8 (slave, 0x1607, 0, 0);
    wsum += set_pdo_write32(slave, 0x1607, 1, 0x60400010); // Control word
    wsum += set_pdo_write32(slave, 0x1607, 2, 0x607A0020); // Target position
    //wsum += set_pdo_write32(slave, 0x1607, 3, 0x60B10020); // Velocity offset
    //wsum += set_pdo_write32(slave, 0x1607, 4, 0x60B20010); // torque offset
    wsum += set_pdo_write32(slave, 0x1607, 3, 0x60600008); // mode of operation
    wsum += set_pdo_write32(slave, 0x1607, 4, 0x00000008); // mode of operation
    // wsum += set_pdo_write32(slave, 0x1607, 4, 0x60710010); // Target torque
    wsum += set_pdo_write8 (slave, 0x1607, 0, 6);
    wsum += set_pdo_write16(slave, 0x1C12, 1, 0x1607);
    wsum += set_pdo_write8 (slave, 0x1C12, 0, 1);


    wsum += set_pdo_write8 (slave, 0x1C13, 0, 0);
    wsum += set_pdo_write8 (slave, 0x1A07, 0, 0);
    wsum += set_pdo_write32(slave, 0x1A07, 1, 0x60410010); // Status word
    wsum += set_pdo_write32(slave, 0x1A07, 2, 0x60640020); // Position actual value
    //wsum += set_pdo_write32(slave, 0x1A07, 3, 0x606C0020); // Velocity actual value
    //wsum += set_pdo_write32(slave, 0x1A07, 4, 0x60770010); // torque offset
    wsum += set_pdo_write32(slave, 0x1A07, 3, 0x60610008); // mode of operation display
    wsum += set_pdo_write32(slave, 0x1A07, 4, 0x00000008); // mode of operation display
    //
    wsum += set_pdo_write8 (slave, 0x1A07, 0, 6);
    wsum += set_pdo_write16(slave, 0x1C13, 1, 0x1A07);
    wsum += set_pdo_write8 (slave, 0x1C13, 0, 1);

    return wsum;
}

uint16 ec_set_state_machine(uint16 reqstate, int timeout)
{
   ec_statecheck(0, reqstate,  timeout * 4);
   ec_slave[0].state = reqstate;
   /* send one valid process data to make outputs in slaves happy*/
   ec_send_processdata();
   ec_receive_processdata(EC_TIMEOUTRET);
   /* request OP state for all slaves */
   ec_writestate(0);
   uint16 chk = 10;
   /* wait for all slaves to reach OP state */
   do {
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_statecheck(0, reqstate, timeout);
   } while (chk-- && (ec_slave[0].state != reqstate));
   return chk;
}

void motor_control(char *ifname)
{
   int i, oloop, iloop;
   needlf = FALSE;
   inOP = FALSE;
   uint8_t value = 0;
   uint16 slave = 1;

   printf("Starting motor control test\n");
   /* initialise SOEM, bind socket to ifname */
   if (!ec_init(ifname)) {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
      return;
   } 
   
   printf("ec_init on %s succeeded.\n",ifname);
   /* find and auto-config slaves */
   if (ec_config_init(FALSE) <= 0) {
      printf("No slaves found!\n");
      /* stop SOEM, close socket */
      ec_close();
      return;
   }
      
   printf("%d slaves found and configured.\n", ec_slavecount);
   printf("State: %x\n", ec_slave[0].state);
   ec_config_map(&IOmap);
   ec_configdc();

   printf("Slaves mapped, state to SAFE_OP.\n");
   /* wait for all slaves to reach SAFE_OP state */
   ec_set_state_machine(EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

   oloop = ec_slave[0].Obytes;
   if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
   if (oloop > 8) oloop = 8;
   iloop = ec_slave[0].Ibytes;
   if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
   if (iloop > 8) iloop = 8;

   // printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

   printf("Request operational state for all slaves\n");
   expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
   printf("Calculated workcounter %d\n", expectedWKC);

   ec_set_state_machine(EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

   if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
      printf("Operational state reached for all slaves.\n");
      inOP = TRUE;

      /* motor control */
      uint8  OpertionMode = 8; // CSP mode
      uint16 ControlWord = 0;
      uint32 JointCmdPos = 0;
      // read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words
      set_pdo_write8(slave, 0x6060, 0, OpertionMode); // mode of opertation
      // read_PDO_item(slave, 0x6061, 0, 1, "Read Mode of operaion");    // mode of operation
      
      usleep(100000);
      ControlWord = 128; 
      set_pdo_write16(slave, 0x6040, 0, ControlWord);    // controlwords: CONTROL_WORD_RESET
      // read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

      usleep(100000);
      ControlWord = 6; // 
      set_pdo_write8(slave, 0x6040, 0, ControlWord);      // controlwords: CONTROL_WORD_SHUTDOWN
      //read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

      usleep(100000);
      JointCmdPos = 0;
      set_pdo_write32(slave, 0x607A, 0, JointCmdPos); // position
      // read_PDO_item(slave, 0x60b0, 0, 4, "Read Inc Tar Position");

      usleep(100000);
      ControlWord = 7;
      set_pdo_write16(slave, 0x6040, 0, ControlWord);      // controlwords: CONTROL_WORD_SWITHON
      // read_PDO_item(slave, 0x6041, 0, 2, "Read Statusword");          // state words

      usleep(100000);
      ControlWord = 15;
      set_pdo_write16(slave, 0x6040, 0, ControlWord);     // controlwords: CONTROL_WORD_ENABLE

      for (value = 0; value < 255; ++ value) {
         JointCmdPos = value * 4;
         set_pdo_write32(slave, 0x607A, 0, JointCmdPos); // position
         // read_PDO_item(slave, 0x6064, 0, 4, "Read Current Position");
         osal_usleep(5000);
      }

      for (value = 0; value < 255; ++ value) {
         JointCmdPos = (254 - value) * 4;
         set_pdo_write32(slave, 0x607A, 0, JointCmdPos); // position
         // read_PDO_item(slave, 0x6064, 0, 4, "Read Current Position");
         osal_usleep(5000);
      }
      
   } else {
      printf("Not all slaves reached operational state.\n");
      ec_readstate();
      for (i = 1; i<=ec_slavecount ; i++) {
         if(ec_slave[i].state != EC_STATE_OPERATIONAL) {
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
         }
      }
   }
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
        osal_usleep(100);
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
      motor_control(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}

