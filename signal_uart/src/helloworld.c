#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xgpio.h"
#include "xparameters.h"
#include "xuartlite.h"
#include <unistd.h>
#include <string.h>
#include <xil_types.h>

#define GPIO_BASEADDR XPAR_AXI_UARTLITE_0_BASEADDR
#define REG2_OFFSET 0x4
#define REG3_OFFSET 0x8
#define REG4_OFFSET 0xC
#define REG5_OFFSET 0x10

XUartLite UartLiteInst;
#define UART_DEVICE_ID XPAR_UARTLITE_0_DEVICE_ID
#define PACKET_SIZE 9  // Updated packet size to include start, end, and message length

u8 RecvBuffer[PACKET_SIZE];

// Define acknowledgment/error codes
#define ACK_SUCCESS            0xA1
#define ERR_START_OF_MESSAGE   0xE1
#define ERR_MESSAGE_LENGTH     0xE2
#define ERR_COMMAND_CODE       0xE3
#define ERR_CHECKSUM           0xE4
#define ERR_END_OF_MESSAGE     0xE5

u8 CalculateChecksum(u8* data, int length) {
    u8 checksum = 0;
    for (int i = 2; i < length - 2; i++) {  // Exclude start, end, and checksum byte
        checksum ^= data[i];
    }
    return checksum;
}

int SendAck(u8 ackByte) {
    XUartLite_Send(&UartLiteInst, &ackByte, 1);
    return 0;
}

int ReceiveUartPacket(u8* buffer) {
    for (int i = 0; i < PACKET_SIZE; i++) {
        while (XUartLite_Recv(&UartLiteInst, &buffer[i], 1) != 1);
    }
    return 0;
}

int DecodePacket(u8* packet, u32* pri, u32* pw) {
    // Check start of message
    if (packet[0] != 0xC9) {
        SendAck(ERR_START_OF_MESSAGE);
        return 0;
    }

    // Check end of message
    if (packet[8] != 0xCE) {
        SendAck(ERR_END_OF_MESSAGE);
        return 0;
    }

    // Check message length (expected to be 7)
    if (packet[1] != 0x05) {
        SendAck(ERR_MESSAGE_LENGTH);
        return 0;
    }

    // Check command code (expected to be 0x01)
    if (packet[2] != 0x01) {
        SendAck(ERR_COMMAND_CODE);
        return 0;
    }

    // Check checksum
    u8 received_checksum = packet[7];
    u8 calculated_checksum = CalculateChecksum(packet, PACKET_SIZE);
    if (received_checksum != calculated_checksum) {
        SendAck(ERR_CHECKSUM);
        return 0;
    }

    // Extract PRI and PW values
    *pw = (packet[3] << 8) | packet[4];  // PRI in microseconds
    *pri = (packet[5] << 8) | packet[6];   // PW in microseconds

    // Send success acknowledgment
    SendAck(ACK_SUCCESS);
    
    return 1;
}

int main() {
    int Status;
    u8 received_packet[PACKET_SIZE];
    u32 pri_value  , pw_value ;
    u32 previous_pri = -1, previous_pw = -1;

    init_platform();



    Status = XUartLite_Initialize(&UartLiteInst, XPAR_XUARTLITE_0_BASEADDR);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    XGpio_WriteReg(XPAR_WAVEFORM_IP_0_BASEADDR, REG2_OFFSET, 2 * 100 + 200); // PW cover
    XGpio_WriteReg(XPAR_WAVEFORM_IP_0_BASEADDR, REG3_OFFSET, 100);
    XGpio_WriteReg(XPAR_WAVEFORM_IP_0_BASEADDR, REG4_OFFSET, 500 * 100);      // PRI
    XGpio_WriteReg(XPAR_WAVEFORM_IP_0_BASEADDR, REG5_OFFSET, 2 * 100 + 100);

    while (1) {
        ReceiveUartPacket(received_packet);

        if (DecodePacket(received_packet, &pri_value, &pw_value)) {
            sleep(1);

            // Only update registers if PRI or PW values have changed
            if (pri_value != previous_pri || pw_value != previous_pw) {
                XGpio_WriteReg(XPAR_WAVEFORM_IP_0_BASEADDR, REG2_OFFSET, pw_value * 100 + 200); // PW cover
                XGpio_WriteReg(XPAR_WAVEFORM_IP_0_BASEADDR, REG3_OFFSET, 100);
                XGpio_WriteReg(XPAR_WAVEFORM_IP_0_BASEADDR, REG4_OFFSET, pri_value * 100);      // PRI
                XGpio_WriteReg(XPAR_WAVEFORM_IP_0_BASEADDR, REG5_OFFSET, pw_value * 100 + 100);

                previous_pri = pri_value;
                previous_pw = pw_value;
            }
        }
    }

    cleanup_platform();
    return 0;
}

/* packet format :  start of message + message length + command code + PRI(in microseconds) + PW(in microseconds) + checksum + end of message
                     1 byte          +    1 byte      +  1 byte      +       2 bytes        +        2 bytes      +  1 byte  +    1 byte

acknowlwdgement : 1. Data has been Successfully Received : 0xA1
                  2. Start of Message Error : 0xE1
                  3. Message Length Error : 0xE2
                  4. Command Code Error : 0xE3
                  5. Checksum Error : 0xE4 (Start of Message, End of Message Error and Checksum itself has to be excluded while calculating the checksum value )
                  6. End of Message Error : 0xE5
            
Note: PRI and PW has to be sent in microseconds and and then it will be multiplied by 100 while writing in register
*/
//C9 09 01 000203E8 E1 CE 