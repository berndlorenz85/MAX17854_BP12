#include "Scheduler.h"
#include "MAX17841_Interface/MAX17841_Interface.h"
#include "mbed.h"

//#define EN_TESTMODE_DEBUG

char SM24_Execution(void);
void Readout_Scandone_Fail_Regs(int row, int col);
void Corrupt_OTP_Registers(char waddr, unsigned short wdata);
char BALSW_OPEN_Diagnostic(void);
char SCAN_Execution(void);

extern Serial pc;
Timer timebase;

DigitalOut dbg2(P1_5);
DigitalOut SHDNL(P0_2);

// MCU board status LEDs
DigitalOut rLED(LED1);
DigitalOut gLED(LED2);
DigitalOut bLED(LED3);

#define RGBLED_WHITE                                                           \
  rLED = LED_ON;                                                               \
  bLED = LED_ON;                                                               \
  gLED = LED_ON
#define RGBLED_RED                                                             \
  rLED = LED_ON;                                                               \
  bLED = LED_OFF;                                                              \
  gLED = LED_OFF
#define RGBLED_GREEN                                                           \
  rLED = LED_OFF;                                                              \
  bLED = LED_OFF;                                                              \
  gLED = LED_ON
#define RGBLED_BLUE                                                            \
  rLED = LED_OFF;                                                              \
  bLED = LED_ON;                                                               \
  gLED = LED_OFF
#define RGBLED_PURPLE                                                          \
  rLED = LED_ON;                                                               \
  bLED = LED_ON;                                                               \
  gLED = LED_OFF

// Wakeup Variables
char u5DaisyChainSize =
    0; // Size of the daisy Chain (will be written at HELLOALL)
unsigned long waketime = 0; // Time the chain takes to wake up (in us)

//  Incrementing pointer goes left to right and then continuing next
//  ReadALL fills all devices, so we have to select the ROW for the parameter
//  and the column increments automatically Param1: D0[0][0] D1[0][1] D2[0][2]
//  D3[0][3] Param2: D0[1][0] D1[1][1] D2[1][2] D3[1][3] Param3: D0[2][0]
//  D1[2][1] D2[2][2] D3[2][3]

short ConfigData[153][32];      //[Number of Parameters][Number of devices]
short readAllReadback[153][32]; //[Number of Parameters][Number of devices]

int loopcounter, testmode;
short MinCell[32], MaxCell[32];

char Scheduler_Init(void)
{
    int row, col;

    // Initialization: WHITE
    RGBLED_WHITE;

    timebase.start();

    SHDNL = true;

    if (MAX17841_Init(&waketime, &u5DaisyChainSize) != 0x00) 
    {
        pc.printf("Daisy Chain Initialization failed\r\n");
        return 1;
    } 
    else 
    {
        pc.printf("Chain woke up in %u Microseconds\r\n", waketime);
        pc.printf("MAX17851 Initialized. Chain Size = %d\r\n", u5DaisyChainSize);

        if (u5DaisyChainSize == 0) 
        {
            pc.printf("Not Initialized properly\n\r");
            SHDNL = false;
            wait_ms(100);
            Scheduler_Init();
        }

        MAX17852_writeAll('M', DEVCFG1, 0x0108, 200); // Single UART, UART HOST, DC Enable
        MAX17852_writeAll_RB('M');

        // GoTo Testmode
        pc.printf("TM_EN\n\r");
        MAX17852_writeAll('M', 0xC0, 0x0012, 200);
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', 0xC0, 0x0034, 200);
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', 0xC0, 0x00FA, 200);
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', 0xC0, 0x00DB, 200);
        MAX17852_writeAll_RB('M');
        MAX17852_readAll('M', 0xC1);                                //Readback OTP
        wait_us(600);
        MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);        //Get data of RB
        pc.printf("R:%02x", 0xC1);                       //Print register to console
        for (col = 0; col < u5DaisyChainSize; col++) 
        {
            pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
        }

        // CONFIG Readout-------------------------------------------------
        pc.printf("After_Reset:\r\n");
        pc.printf("------------\r\n");
        Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing

        MAX17852_writeAll('M', PACKCFG, 0x0FFF, 200); // Disable Flexpack
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', AUXGPIOCFG, 0x003C, 200); // Enable ADIO0/1
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', MEASUREEN1, 0x003F, 200); // Enable All Cells but no block
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', MEASUREEN2, 0x003F, 200); // enable AUX
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', ADCTEST1AREG, 0x0100, 200); // Set ADCTEST registers
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', ADCTEST1BREG, 0x0101, 200); // Set ADCTEST registers
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', ADCTEST2AREG, 0x0102, 200); // Set ADCTEST registers
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', ADCTEST2BREG, 0x0103, 200); // Set ADCTEST registers
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', DIAGCFG, 0xF012, 200); // enable Diagnostic (TEMP & VAA)
        MAX17852_writeAll_RB('M');

        /*
        for (row = 0; row < 0x98; row++) {
            MAX17852_readAll('M', (short)row);
            wait_us(600);
            MAX17852_readAll_RB('M', &ConfigData[row][0]);
        }
        pc.printf("REG DEVn..DEV1 DEV0\r\n");

        for (row = 0; row < 153; row++) {
            pc.printf("%02x", row);
            for (col = 0; col < u5DaisyChainSize; col++) {
            pc.printf(" %04x", (unsigned short)ConfigData[row][col]);
            }
            pc.printf("\r\n");
        }*/

        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', STATUS1, 0x0000, 200); // Clear ALRTRESET
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', STATUS2, 0x0000, 200); // Clear Status
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', STATUS3, 0x0000, 200); // Clear Status

        // CONFIG Readout-------------------------------------------------
        pc.printf("After_Config:\r\n");
        pc.printf("-------------\r\n");
        Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing

        /* GoTo Testmode
        MAX17852_writeAll('M', 0xC0, 0x0012, 200);
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', 0xC0, 0x0034, 200);
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', 0xC0, 0x00FA, 200);
        MAX17852_writeAll_RB('M');
        MAX17852_writeAll('M', 0xC0, 0x00DB, 200);
        MAX17852_writeAll_RB('M');*/

        // SCAN_Execution();
        // SM24_Execution();
        // BALSW_OPEN_Diagnostic();

        // Initialization Done OK: GREEN
        RGBLED_GREEN;

        loopcounter = 0;
        testmode = 0;
        pc.printf("Loop_Start\r\n");
        return 0;
    }
}

void Scheduler_Loop(void) 
{
    int row, col;

    // Continuous
    // SCAN-------------------------------------------------------------------------------
    MAX17852_writeAll('M', DIAGCFG, 0x0012, 200); // DIAG1 = Die Temp, DIAG2 = VAADIAG
    MAX17852_writeAll_RB('M');
    MAX17852_writeAll('M', SCANCTRL, 0x0011, 200); // SCAN: OS8
    MAX17852_writeAll_RB('M');
    wait_us(3000);
    // Readback Status
    for (row = 0; row < 0x12; row++) 
    {
        MAX17852_readAll('M', (short)row);
        wait_us(600);
        MAX17852_readAll_RB('M', &readAllReadback[row][0]); // STATUS = 0x00 - 0x12
    }
    // Readback SCANCTRL
    MAX17852_readAll('M', SCANCTRL);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x13][0]); // SCANCTRL = 0x13
    // Readback CELL1-6
    for (row = 0; row < 6; row++)
    {
        MAX17852_readAll('M', CELL1REG + row);
        wait_us(600);
        MAX17852_readAll_RB('M', &readAllReadback[row + 0x14][0]); // CELL = 0x14 - 0x19
    }
    // Readback DIAG AUX
    for (row = 0; row < 4; row++) 
    {
        MAX17852_readAll('M', DIAG1REG + row);
        wait_us(600);
        MAX17852_readAll_RB('M', &readAllReadback[row + 0x1A][0]); // CELL = 0x1A - 0x1D
    }

    // Going through all devices in the Daisy Chain and find the faulty SCANCTRL -
    // if so, report it and re-init
    for (col = 0; col < u5DaisyChainSize; col++) 
    {
        if (((readAllReadback[0x13][col] & 0xA000) != 0xA000) || ((readAllReadback[0x13][col] == 0x0000))) 
        {
            //pc.printf("Device_%d corrupt_UncorruptD2DoubleScan\n\r", col);
            pc.printf("Fail: 0x66 = %x\n\r", readAllReadback[0x13][col]);
            Readout_Scandone_Fail_Regs(row, col);
            SHDNL = false;
            wait_ms(100);
            Scheduler_Init();


            //MAX17852_writeAll('M', SWACTION, 0x0001, 200); // DIAG1 = Die Temp, DIAG2 = VAADIAG
            //MAX17852_writeAll_RB('M');            
            //MAX17852_writeAll('M', 0xD2, 0x0C3F , 200);
            //MAX17852_writeAll_RB('M');
            //MAX17852_writeAll('M', 0x66, 0x0011 , 200);
            //MAX17852_writeAll_RB('M');
            //MAX17852_writeAll('M', 0x66, 0x0011 , 200);
            //MAX17852_writeAll_RB('M');
            //SHDNL = true;
            //Readout_Scandone_Fail_Regs(row, col);
            //SHDNL = false;
            //wait_ms(100);
            //SHDNL = true;
            //Scheduler_Init();
            //Readout_Scandone_Fail_Regs(row, col);
            //SHDNL = false;
            //wait_ms(100);
            //SHDNL = true;
            //Scheduler_Init();
            //Readout_Scandone_Fail_Regs(row, col);
            //break;
        }
    }
    wait_ms(100);

    switch (loopcounter++) 
    {
        case 100:
            pc.printf("%d\n\r",loopcounter);
            Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing
            break;
        case 200:
            pc.printf("%d\n\r",loopcounter);
            /*pc.printf("Corruption\n\r");
            MAX17852_writeAll('M', 0xD2, 0x0C00 , 200);
            MAX17852_writeAll_RB('M');*/
            Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing
            break;
        case 300:
            pc.printf("%d\n\r",loopcounter);
            Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing
            break;
        case 400:
            pc.printf("%d\n\r",loopcounter);
            Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing
            break;
        case 500:
            pc.printf("%d\n\r",loopcounter);
            Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing
            break;
        case 600:
            pc.printf("%d\n\r",loopcounter);
            Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing
            break;
        case 700:
            pc.printf("%d\n\r",loopcounter);
            Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing
            break;
        case 800:
            pc.printf("%d\n\r",loopcounter);
            Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing
            break;
        case 900:
            pc.printf("%d\n\r",loopcounter);
            Readout_Scandone_Fail_Regs(0, 0); // arguments mean nothing
            loopcounter = 0;
            break;
        default:
            break;
    }

    /* Report result after so many steps
    if (++loopcounter >= 1000) {
    pc.printf("Loops: %d\n\r", loopcounter);
    pc.printf("SCAN Result\n\r");
    pc.printf("----\n\r");
    // Readout Results
    for (row = 0; row < 0x1E; row++) {
        if (row <= 0x12) {
        pc.printf("R:%02x", (unsigned short)(row));
        } else if (row == 0x13) {
        pc.printf("R:%02x", (unsigned short)(row + SCANCTRL - 0x13));
        } else if ((row >= 0x14) && (row <= 0x19)) {
        pc.printf("R:%02x", (unsigned short)(row + CELL1REG - 0x14));
        } else if ((row >= 0x1A) && (row <= 0x1D)) {
        pc.printf("R:%02x", (unsigned short)(row + DIAG1REG - 0x1A));
        } else {
        }
        for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[row][col]);
        }
        pc.printf("\n\r");
    }*/

    // BALSW_OPEN_Diagnostic();

#ifdef EN_TESTMODE_DEBUG
    pc.printf("Entering Testmode\n\r");
    pc.printf("-----------------");
    // GoTo Testmode
    MAX17852_writeAll('M', 0xC0, 0x0012, 200);
    MAX17852_writeAll_RB('M');
    MAX17852_writeAll('M', 0xC0, 0x0034, 200);
    MAX17852_writeAll_RB('M');
    MAX17852_writeAll('M', 0xC0, 0x00FA, 200);
    MAX17852_writeAll_RB('M');
    MAX17852_writeAll('M', 0xC0, 0x00DB, 200);
    MAX17852_writeAll_RB('M');
    // check if in Testmode
    MAX17852_readAll('M', 0xC1);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
    pc.printf("\n\rR:C1");
    for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
    }
    pc.printf("\n\r-----------------");
    // Readback Interesting Registers
    MAX17852_readAll('M', 0xD2);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
    pc.printf("\n\rR:D2");
    for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
    }
    MAX17852_readAll('M', 0xE0);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
    pc.printf("\n\rR:E0");
    for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
    }

    // Testing out the SCANDONE failure on the next BALSW Diagnostic!!!
    if (testmode == 0) {
        // Set bit for overwriting trim
        MAX17852_writeAll('M', 0xE0, 0x0001, 200);
        MAX17852_writeAll_RB('M');
        // set trim
        MAX17852_writeAll('M', 0xE0, 0x6001, 200);
        MAX17852_writeAll_RB('M');
        MAX17852_readAll('M', 0xE0);
        wait_us(600);
        MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
        pc.printf("\n\rR:E0");
        for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
        }
        MAX17852_readAll('M', 0xD0);
        wait_us(600);
        MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
        pc.printf("\n\rR:D0");
        for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
        }
        MAX17852_readAll('M', 0xD2);
        wait_us(600);
        MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
        pc.printf("\n\rR:D2");
        for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
        }
        testmode++;

        BALSW_OPEN_Diagnostic();
    } else {
        // Disable Trim overwrite
        MAX17852_writeAll('M', 0xE0, 0xFFFE, 200);
        MAX17852_writeAll_RB('M');
        testmode = 0;
    }

    MAX17852_readAll('M', 0xE0);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
    pc.printf("\n\rR:E0");
    for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
    }
    MAX17852_readAll('M', 0xD0);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
    pc.printf("\n\rR:D0");
    for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
    }
    MAX17852_readAll('M', 0xD2);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
    pc.printf("\n\rR:D2");
    for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
    }

    // Exit Testmode
    pc.printf("\n\rExit Testmode");
    pc.printf("\n\r-------------");
    MAX17852_writeAll('M', 0xC0, 0x0001, 200);
    MAX17852_writeAll_RB('M');
    // Check if out of Testmode
    MAX17852_readAll('M', 0xC1);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
    pc.printf("\n\rR:C1");
    for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][col]);
    }
    pc.printf("\n\r-------------\n\r");
#endif

}

char SCAN_Execution(void) {
    int row, col;

    pc.printf("SCAN\n\r");
    pc.printf("----\n\r");
    MAX17852_writeAll('M', DIAGCFG, 0x0012,
                    200); // DIAG1 = Die Temp, DIAG2 = VAADIAG
    MAX17852_writeAll_RB('M');
    MAX17852_writeAll('M', SCANCTRL, 0x0011, 200); // SCAN: OS8
    MAX17852_writeAll_RB('M');
    wait_us(3000);
    // Readback Status
    for (row = 0; row < 0x12; row++) {
    MAX17852_readAll('M', (short)row);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[row][0]); // STATUS = 0x00 - 0x12
    }
    // Readback SCANCTRL
    MAX17852_readAll('M', SCANCTRL);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x13][0]); // SCANCTRL = 0x13
    // Readback CELL1-6
    for (row = 0; row < 7; row++) {
    MAX17852_readAll('M', CELL1REG + row);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[row + 0x14][0]); // CELL = 0x14 - 0x19
    }
    // Readback DIAG AUX
    for (row = 0; row < 4; row++) {
    MAX17852_readAll('M', DIAG1REG + row);
    wait_us(600);
    MAX17852_readAll_RB('M',
                        &readAllReadback[row + 0x1A][0]); // CELL = 0x1A - 0x1D
    }
    // Going through all devices in the Daisy Chain and find the faulty SCANCTRL -
    // if so, report it and re-init
    for (col = 0; col < u5DaisyChainSize; col++) {
    if (((readAllReadback[0x13][col] & 0xA000) != 0xA000) || ((readAllReadback[0x13][col] == 0x0000))) 
    {
        Readout_Scandone_Fail_Regs(row, col);
        SHDNL = false;
        wait_ms(100);
        Scheduler_Init();
        break;
    }
    }
    // Readout Results
    for (row = 0; row < 0x1E; row++) {
    if (row <= 0x12) {
        pc.printf("R:%02x", (unsigned short)(row));
    } else if (row == 0x13) {
        pc.printf("R:%02x", (unsigned short)(row + SCANCTRL - 0x13));
    } else if ((row >= 0x14) && (row <= 0x19)) {
        pc.printf("R:%02x", (unsigned short)(row + CELL1REG - 0x14));
    } else if ((row >= 0x1A) && (row <= 0x1D)) {
        pc.printf("R:%02x", (unsigned short)(row + DIAG1REG - 0x1A));
    } else {
    }
    for (col = 0; col < u5DaisyChainSize; col++) {
        pc.printf(" %04x", (unsigned short)readAllReadback[row][col]);
    }
    pc.printf("\n\r");
    }
    return 0;
}

char BALSW_OPEN_Diagnostic(void) {
  int row, col;

  // BALSW OPEN DIAGNOSTIC-----------------------------------------------
  pc.printf("BALSW OPEN Diagnostic\n\r");
  pc.printf("---------------------\n\r");
  MAX17852_writeAll('M', BALSWCTRL, 0x003F, 200);
  MAX17852_writeAll_RB('M');
  // MAX17852_writeAll('M', BALLOWTHRREG, 0x7F00, 200);
  // MAX17852_writeAll('M', BALHIGHTHRREG, 0x8100, 200);
  MAX17852_writeAll('M', SCANCTRL, 0x0141, 200); // BALWSW OPEN Diagnostic SCAN
  MAX17852_writeAll_RB('M');
  wait_us(3000);
  // Readback Status
  for (row = 0; row < 0x12; row++) {
    MAX17852_readAll('M', (short)row);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[row][0]); // STATUS = 0x00 - 0x12
  }
  // Readback SCANCTRL
  MAX17852_readAll('M', SCANCTRL);
  wait_us(600);
  MAX17852_readAll_RB('M', &readAllReadback[0x13][0]); // SCANCTRL = 0x13
  // Readback CELL1-6
  for (row = 0; row < 6; row++) {
    MAX17852_readAll('M', CELL1REG + row);
    wait_us(600);
    MAX17852_readAll_RB('M',
                        &readAllReadback[row + 0x14][0]); // CELL = 0x14 - 0x19
  }
  // Readback DIAG AUX
  for (row = 0; row < 4; row++) {
    MAX17852_readAll('M', DIAG1REG + row);
    wait_us(600);
    MAX17852_readAll_RB(
        'M', &readAllReadback[row + 0x1A][0]); // DIAG & AUX0/1 = 0x1A - 0x1D
  }
  // Going through all devices in the Daisy Chain and find the faulty SCANCTRL -
  // if so, report it and re-init
  for (col = 0; col < u5DaisyChainSize; col++) {
    if (((readAllReadback[0x13][col] & 0xA000) != 0xA000) ||
        ((readAllReadback[0x13][col] == 0x0000))) {
      Readout_Scandone_Fail_Regs(row, col);
      SHDNL = false;
      wait_ms(100);
      Scheduler_Init();
      break;
    }
  }
  // Readout Results
  for (row = 0; row < 0x1E; row++) {
    if (row <= 0x12) {
      pc.printf("R:%02x", (unsigned short)(row));
    } else if (row == 0x13) {
      pc.printf("R:%02x", (unsigned short)(row + SCANCTRL - 0x13));
    } else if ((row >= 0x14) && (row <= 0x19)) {
      pc.printf("R:%02x", (unsigned short)(row + CELL1REG - 0x14));
    } else if ((row >= 0x1A) && (row <= 0x1D)) {
      pc.printf("R:%02x", (unsigned short)(row + DIAG1REG - 0x1A));
    } else {
    }
    for (col = 0; col < u5DaisyChainSize; col++) {
      pc.printf(" %04x", (unsigned short)readAllReadback[row][col]);
    }
    pc.printf("\n\r");
  }
  return 0;
}

char SM24_Execution(void) {
  int row, col;

  pc.printf("SM24 Execution\n\r");
  pc.printf("--------------\n\r");
  MAX17852_writeAll('M', DIAGCFG, 0x00AB,
                    200); // DIAG1 = ADC Zero Scale, DIAG2 = ADC Full Scale
  MAX17852_writeAll_RB('M');
  MAX17852_writeAll('M', ADCTEST1AREG, 0x8100, 200);
  MAX17852_writeAll_RB('M');
  MAX17852_writeAll('M', SCANCTRL, 0x0001, 200); // SCAN: OS1
  MAX17852_writeAll_RB('M');
  wait_us(3000);
  // Readback Status
  for (row = 0; row < 0x12; row++) {
    MAX17852_readAll('M', (short)row);
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[row][0]); // STATUS = 0x00 - 0x12
  }
  // Readback SCANCTRL
  MAX17852_readAll('M', SCANCTRL);
  wait_us(600);
  MAX17852_readAll_RB('M', &readAllReadback[0x13][0]); // SCANCTRL = 0x13
  // Readback CELL1-6
  for (row = 0; row < 6; row++) {
    MAX17852_readAll('M', CELL1REG + row);
    wait_us(600);
    MAX17852_readAll_RB('M',
                        &readAllReadback[row + 0x14][0]); // CELL = 0x14 - 0x19
  }
  // Readback DIAG AUX
  for (row = 0; row < 4; row++) {
    MAX17852_readAll('M', DIAG1REG + row);
    wait_us(600);
    MAX17852_readAll_RB('M',
                        &readAllReadback[row + 0x1A][0]); // CELL = 0x1A - 0x1D
  }
  // Going through all devices in the Daisy Chain and find the faulty SCANCTRL -
  // if so, report it and re-init
  for (col = 0; col < u5DaisyChainSize; col++) {
    if (((readAllReadback[0x13][col] & 0xA000) != 0xA000) ||
        ((readAllReadback[0x13][col] == 0x0000))) {
      Readout_Scandone_Fail_Regs(row, col);
      SHDNL = false;
      wait_ms(100);
      Scheduler_Init();
      break;
    }
  }
  // Readout Results
  for (row = 0; row < 0x1E; row++) {
    if (row <= 0x12) {
      pc.printf("R:%02x", (unsigned short)(row));
    } else if (row == 0x13) {
      pc.printf("R:%02x", (unsigned short)(row + SCANCTRL - 0x13));
    } else if ((row >= 0x14) && (row <= 0x19)) {
      pc.printf("R:%02x", (unsigned short)(row + CELL1REG - 0x14));
    } else if ((row >= 0x1A) && (row <= 0x1D)) {
      pc.printf("R:%02x", (unsigned short)(row + DIAG1REG - 0x1A));
    } else {
    }
    for (col = 0; col < u5DaisyChainSize; col++) {
      pc.printf(" %04x", (unsigned short)readAllReadback[row][col]);
    }
    pc.printf("\n\r");
  }
  MAX17852_writeAll('M', ADCTEST1AREG, 0x0100, 200);
  MAX17852_writeAll_RB('M');
  return 0;
}

void Corrupt_OTP_Registers(char waddr, unsigned short wdata)
{
    MAX17852_writeAll('M', 0xC3, (unsigned short)0x20, 200);        //Set MR to Override
    MAX17852_writeAll_RB('M');        
    MAX17852_writeAll('M', 0xC4, (unsigned short)waddr, 200);     //Set OTP Address
    MAX17852_writeAll_RB('M');
    MAX17852_writeAll('M', 0xC5, (unsigned short)wdata, 200);       //Write data to address
    MAX17852_writeAll_RB('M');
    MAX17852_writeAll('M', 0xC7, (unsigned short)0x00, 200);        //Clear Mask
    MAX17852_writeAll_RB('M');
    MAX17852_writeAll('M', 0xC2, (unsigned short)0x43, 200);        //VDD_ON, PROT
    MAX17852_writeAll_RB('M');
    pc.printf("WOTP: %02x Val: %02x\n\r", waddr, wdata);          //Print
}

void Readout_Scandone_Fail_Regs(int row, int col) 
{
    unsigned char regRead;
    int coll; // local variable

    for (regRead = 0x00; regRead <= 0x9D; regRead++)            //Read normal memory
    {
        MAX17852_readAll('M', regRead);                         //Readout READREG
        wait_us(600);                                           //Wait for message
        MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);    //Get Reg Data
        pc.printf("\n\rR:%02x", regRead);                       //Print register to console
        for (coll = 0; coll < u5DaisyChainSize; coll++) {       //Print each device
            pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][coll]);
        }
    }

    for (regRead = 0xC0; regRead <= 0xE0; regRead++)            //Read normal memory
    {
        MAX17852_readAll('M', regRead);                         //Readout READREG
        wait_us(600);                                           //Wait for message
        MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);    //Get Reg Data
        pc.printf("\n\rR:%02x", regRead);                       //Print register to console
        for (coll = 0; coll < u5DaisyChainSize; coll++) {       //Print each device
            pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][coll]);
        }
    }

    //ADC Testmode Readout (0x00 - 0x24)
    for (regRead = 0x00; regRead <= 0x24; regRead++) 
    {
        MAX17852_writeAll('M', 0xD2, ((unsigned short)regRead<<8) + 0x8000, 200);
        MAX17852_writeAll_RB('M');
        MAX17852_readAll('M', 0xD2);
        wait_us(600);
        MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);
        pc.printf("\n\rATR:%02x", regRead);
        for (coll = 0; coll < u5DaisyChainSize; coll++) 
        {
            pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][coll]);
        }
    }

    //OTP2 Readout (0x00 - 0x4E)
    for (regRead = 0x00; regRead <= 0x4E; regRead++)
    {
        MAX17852_writeAll('M', 0xC4, (unsigned short)regRead, 200);     //Set OTP Address
        MAX17852_writeAll_RB('M');                                      //Readback
        MAX17852_readAll('M', 0xC6);                                    //Readback OTP
        wait_us(600);
        MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);            //Get data of RB
        pc.printf("\n\rOTR:%02x", regRead);                             //Print
        for (coll = 0; coll < u5DaisyChainSize; coll++) 
        {
            pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][coll]);
        }
    }
    pc.printf("\n\r");
/*
    pc.printf("TM_DIS\n\r");
    MAX17852_writeAll('M', 0xC0, 0x0000, 200);
    MAX17852_writeAll_RB('M');
    MAX17852_readAll('M', 0xC1);                                //Readback OTP
    wait_us(600);
    MAX17852_readAll_RB('M', &readAllReadback[0x1F][0]);        //Get data of RB
    pc.printf("R:%02x", 0xC1);                       //Print register to console
    for (coll = 0; coll < u5DaisyChainSize; coll++) 
    {
        pc.printf(" %04x", (unsigned short)readAllReadback[0x1F][coll]);
    }
    pc.printf("\n\r");*/
}
