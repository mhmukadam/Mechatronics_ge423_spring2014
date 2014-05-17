//McBSP1
#define DRR1 *(volatile unsigned int *)(0x01D11000)  //McBSP Data Receive Register (read-only)
#define DXR1 *(volatile unsigned int *)(0x01D11004)  //McBSP Data Transmit Register
#define SPCR1 *(volatile unsigned int *)(0x01D11008)  //McBSP Serial Port Control Register
#define RCR1 *(volatile unsigned int *)(0x01D1100C)  //McBSP Receive Control Register
#define XCR1 *(volatile unsigned int *)(0x01D11010)  //McBSP Transmit Control Register
#define SRGR1 *(volatile unsigned int *)(0x01D11014)  //McBSP Sample Rate Generator register
#define MCR1 *(volatile unsigned int *)(0x01D11018)  //McBSP Multichannel Control Register
#define RCERE1_0 *(volatile unsigned int *)(0x01D1101C)  //McBSP Enhanced Receive Channel Enable Register 0 Partition A/B
#define XCERE1_0 *(volatile unsigned int *)(0x01D11020)  //McBSP Enhanced Transmit Channel Enable Register 0 Partition A/B
#define PCR1 *(volatile unsigned int *)(0x01D11024)  //McBSP Pin Control Register
#define RCERE1_1 *(volatile unsigned int *)(0x01D11028)  //McBSP Enhanced Receive Channel Enable Register 1 Partition C/D
#define XCERE1_1 *(volatile unsigned int *)(0x01D1102C)  //McBSP Enhanced Transmit Channel Enable Register 1 Partition C/D
#define RCERE1_2 *(volatile unsigned int *)(0x01D11030)  //McBSP Enhanced Receive Channel Enable Register 2 Partition E/F
#define XCERE1_2 *(volatile unsigned int *)(0x01D11034)  //McBSP Enhanced Transmit Channel Enable Register 2 Partition E/F
#define RCERE1_3 *(volatile unsigned int *)(0x01D11038)  //McBSP Enhanced Receive Channel Enable Register 3 Partition G/H
#define XCERE1_3 *(volatile unsigned int *)(0x01D1103C)  //McBSP Enhanced Transmit Channel Enable Register 3 Partition G/H
//McBSP FIFO Control and Status Registers
#define BFIFOREV1 *(volatile unsigned int *)(0x01D11800)  //BFIFO Revision Identification Register
#define WFIFOCTL1 *(volatile unsigned int *)(0x01D11810)  //Write FIFO Control Register
#define WFIFOSTS1 *(volatile unsigned int *)(0x01D11814)  //Write FIFO Status Register
#define RFIFOCTL1 *(volatile unsigned int *)(0x01D11818)  //Read FIFO Control Register
#define RFIFOSTS1 *(volatile unsigned int *)(0x01D1181C)  //Read FIFO Status Register
//McBSP FIFO Data Registers
#define RBUF1 *(volatile unsigned int *)(0x01F11000) //McBSP FIFO Receive Buffer
#define XBUF1 *(volatile unsigned int *)(0x01F11000) //McBSP FIFO Transmit Buffer
#define RBUF1_ADDRESS 0x01F11000

void init_McBSP(void);
void send_McBSP(void);
void SetRobotOutputs(float vref, float turn, float pwm3, float pwm4, float pwm5, float pwm3b, float pwm4b, float reserved, float dac1, float dac2);
void SetRobotOutputs_manual(float pwm1, float pwm2, float pwm3, float pwm4, float pwm5, float pwm3b, float pwm4b, float reserved, float dac1, float dac2);
