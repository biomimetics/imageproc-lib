#include "uart_driver.h"
#include "uart.h"
#include "mac_packet.h"
#include "payload.h"
#include "ppool.h"
#include "utils.h"

static MacPacket tx_packet = NULL;
static Payload tx_payload = NULL;
static unsigned char tx_idx;
static unsigned char tx_checksum;

static MacPacket rx_packet = NULL;
static Payload rx_payload = NULL;
static unsigned char rx_idx;
static unsigned char rx_checksum;

static packet_callback rx_callback = NULL;

void uartInit(packet_callback rx_cb) {
    /// UART2 for RS-232 w/PC @ 230400, 8bit, No parity, 1 stop bit
    unsigned int U2MODEvalue, U2STAvalue, U2BRGvalue;
    U2MODEvalue = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
                  UART_MODE_SIMPLEX & UART_UEN_00 & UART_DIS_WAKE &
                  UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE &
                  UART_BRGH_FOUR & UART_NO_PAR_8BIT & UART_1STOPBIT;
    U2STAvalue  = UART_INT_TX & UART_INT_RX_CHAR &UART_SYNC_BREAK_DISABLED &
                  UART_TX_ENABLE & UART_ADR_DETECT_DIS &
                  UART_IrDA_POL_INV_ZERO; // If not, whole output inverted.
    U2BRGvalue  = 9; // =3 for 2.5M Baud
    //U2BRGvalue  = 43; // =43 for 230500Baud (Fcy / ({16|4} * baudrate)) - 1
    //U2BRGvalue  = 86; // =86 for 115200 Baud
    //U2BRGvalue  = 1041; // =1041 for 9600 Baud
    

    OpenUART2(U2MODEvalue, U2STAvalue, U2BRGvalue);

    tx_idx = UART_TX_IDLE;
    rx_idx = UART_RX_IDLE;
    rx_callback = rx_cb;

    ConfigIntUART2(UART_TX_INT_EN & UART_TX_INT_PR5 & UART_RX_INT_EN & UART_RX_INT_PR6);
}

//General blocking UART send function, appends basic checksum
unsigned char uartSend(unsigned char length, unsigned char *frame) {
    int i;
    unsigned char checksum = 0;

    while(BusyUART2());
    WriteUART2(length);
    while(BusyUART2());
    WriteUART2(~length);

    checksum = 0xFF;

    //send payload data
    for (i = 0; i < length; i++) {
        checksum += frame[i];
        while(BusyUART2());
        WriteUART2(frame[i]);
    }

    //Send Checksum Data
    while(BusyUART2());
    WriteUART2(checksum);
    return 1;
}

unsigned char uartSendPayload(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
    MacPacket packet;
    Payload pld;

    packet = ppoolRequestFullPacket(length);
    if(packet == NULL)
        return 0;

    pld = packet->payload;
    paySetType(pld, type);
    paySetStatus(pld, status);
    paySetData(pld, length, frame);
    if(uartSendPacket(packet)) {
        return 1;
    } else {
        ppoolReturnFullPacket(packet);
        return 0;
    }
}

unsigned char uartSendPacket(MacPacket packet) {
    LED_3 = 1;
    if(tx_packet != NULL) {
        ppoolReturnFullPacket(tx_packet);
        tx_packet = NULL;
        tx_idx = UART_TX_IDLE;
    }

    if(tx_idx == UART_TX_IDLE && packet != NULL && packet->payload_length < UART_MAX_SIZE) {
        tx_packet = packet;
        tx_payload = packet->payload;
        tx_checksum = packet->payload_length + 3; // add three for size, size check, and checksum
        tx_idx = UART_TX_SEND_SIZE;
        U2TXREG = tx_checksum;
        LED_3 = 0;
        return 1;
    } else {
        LED_3 = 0;
        return 0;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
    unsigned char tx_byte;

    LED_3 = 1;
    if(tx_idx != UART_TX_IDLE) {
        if(tx_idx == UART_TX_SEND_SIZE) {
            tx_idx = 0;
            tx_byte = ~tx_checksum; // send size check
        } else if(tx_idx == tx_payload->data_length + PAYLOAD_HEADER_LENGTH) {
            ppoolReturnFullPacket(tx_packet);
            tx_packet = NULL;
            tx_idx = UART_TX_IDLE;
            tx_byte = tx_checksum;
        } else {
            tx_byte = tx_payload->pld_data[tx_idx++];
        }
        tx_checksum += tx_byte;
        U2TXREG = tx_byte;
    }
    _U2TXIF = 0;
    LED_3 = 0;
}

//read data from the UART, and call the proper function based on the Xbee code
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    unsigned char rx_byte;

    LED_3 = 1;

    while(U2STAbits.URXDA) {
        rx_byte = U2RXREG;

        if(rx_idx == UART_RX_IDLE && rx_byte < UART_MAX_SIZE) {
            rx_checksum = rx_byte;
            rx_idx = UART_RX_CHECK_SIZE;
        } else if(rx_idx == UART_RX_CHECK_SIZE) {
            if((rx_checksum ^ rx_byte) == 0xFF && rx_checksum < UART_MAX_SIZE) {
                rx_packet = ppoolRequestFullPacket(rx_checksum - (PAYLOAD_HEADER_LENGTH+3));
                rx_payload = rx_packet->payload;
                rx_checksum += rx_byte;
                rx_idx = 0;
            } else {
                rx_checksum = rx_byte;
            }
        } else if (rx_idx == rx_payload->data_length + PAYLOAD_HEADER_LENGTH) {
            if(rx_checksum == rx_byte && rx_callback != NULL) {
                (rx_callback)(rx_packet);
            } else {
                ppoolReturnFullPacket(rx_packet);
            }
            rx_idx = UART_RX_IDLE;
        } else {
            rx_checksum += rx_byte;
            rx_payload->pld_data[rx_idx++] = rx_byte;
        }
    }

    if(U2STAbits.OERR) {
        U2STAbits.OERR = 0;
    }
    
    _U2RXIF = 0;
    LED_3 = 0;
}
