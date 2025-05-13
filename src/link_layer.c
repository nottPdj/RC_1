// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


#define FLAG 0x7E      /* Synchronisation: start or end of frame */
#define A_TX 0x03      /* Address field in frames that are commands sent by the Transmitter or replies sent by the Receiver */
#define A_RX 0x01      /* Address field in frames that are commands sent by the Receiver or replies sent by the Transmitter */

// CONTROL FIELD to indicate the type of supervision frame/message
#define SET 0x03    /* SET frame: sent by the transmitter to initiate a connection */
#define UA 0x07        /* UA frame: confirmation to the reception of a valid supervision frame */
#define DISC 0x0B       /* DISC frame: to indicate the termination of a connection */
#define INF_0 0x00 // Information frame number 0
#define INF_1 0x80 // Information frame number 1, could be 0x40
#define INF_(n) ( (n) == 0 ? INF_0 : INF_1 )
#define ESC 0x7D // Byte stuffing escape octet
#define RR0 0xAA
#define RR1 0xAB
#define RR(n) ( (n) == 0 ? RR0 : RR1 )
#define REJ0 0x54
#define REJ1 0x55
#define REJ(n) ( (n) == 0 ? REJ0 : REJ1 )

#define NEXT_FRAME(f) ( ((f) + 1) % 2)


unsigned char frame_expected = 0;

// COMMS STATISTICS
typedef struct {
    unsigned int frames;
    unsigned int retransmissions;
} comms_stats;

comms_stats stats;

typedef enum {
	START_RCV,      /* Start of the receiving process */
	FLAG_OK,        /* Start flag ok */
	A_OK,           /* Address field ok */
    A_OK_RX,        /* Address field ok (commands from rx and replies from tx) */
	C_OK,           /* Control field ok */
	BCC_OK,         /* BCC field ok */
	STOP_RCV        /* Stop the receiving process */
} state_t;

LinkLayer connectionParams;


int frame_to_send = 0;


int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

void enableAlarm(int time) {
    alarm(time);
    alarmEnabled = TRUE;
}

void clearAlarm() {
    alarm(0);
    alarmCount = 0;
}


void printStatistics() {
    printf("Number of frames = %d\n", stats.frames);
    printf("Number of retransmissions = %d\n", stats.retransmissions);
}


int receiveSupervision(unsigned char a, unsigned char c, int timeout) {
    char byte;

    state_t state = START_RCV;

    while (state != STOP_RCV) {
		
        if (readByte(&byte) == 1) {
           // printf("Read byte = 0x%02X\n", byte);

            
            switch (state) {
                
                case START_RCV:
                    if (byte == FLAG) 
                        state = FLAG_OK;
                    break;
                    
                case FLAG_OK:
                    if (byte == a)
                        state = A_OK;
                    else if (byte != FLAG)
                        state = START_RCV;
                    break;
                    
                case A_OK:
                    if (byte == FLAG)
                        state = FLAG_OK;
                    else if (byte == c)
                        state = C_OK;
                    else 
                        state = START_RCV;
                    break;
                    
                case C_OK:
                    if (byte == FLAG)
                        state = FLAG_OK;
                    else if (byte == (a ^ c))
                        state = BCC_OK;
                    else 
                        state = START_RCV;
                    break;
                    
                case BCC_OK:
                    if (byte == FLAG)
                        state = STOP_RCV;
                    else 
                        state = START_RCV;
                    break;
                    
                default:
                    printf("ERROR: Wrong state (%d) in 'receiving supervision frame' state machine", state);
                    break;
            }

        }
        // Timeout only if sender is waiting for acknowledgement
        if (timeout == TRUE && alarmEnabled == FALSE) {
            stats.retransmissions++;
            printf("Timeout %d", alarmCount);
            return -1;
        }
	}

    printf("--------END OF FRAME-------\n");

    return 0;
}

int sendSupervision(unsigned char a, unsigned char c) {
    // Create frame to send
    char frame[5] = {0};

    frame[0] = FLAG;
    frame[1] = a;
    frame[2] = c;
    frame[3] = a ^ c;
    frame[4] = FLAG;

    // Write the frame until all 5 bytes are written
    while (writeBytes(frame, 5) != 5);

    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    int dl_identifier = openSeialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (dl_identifier < 0) return -1;
    
    connectionParams = connectionParameters;

    (void)signal(SIGALRM, alarmHandler);

    int retransmissions = connectionParameters.nRetransmissions;

    switch (connectionParameters.role)
    {

    case LlTx:
        while (alarmCount < retransmissions){
            sendSupervision(A_TX,SET);
            enableAlarm(connectionParameters.timeout);

            if (receiveSupervision(A_RX, UA,1) == 0) {
                    clearAlarm();
                    printf("Successfully connected!\n");
                    break;
                }

            // Cancel the procedure, maximum number of retransmissions exceeded
            clearAlarm();
            printf("Maximum number of retransmissions exceeded!\n");
            return -1;
        }
        break;

    case LlRx:
         while (1){
            enableAlarm(connectionParameters.timeout);

            if (receiveSupervision(A_TX, SET,0) == 0) {
                    sendSupervision(A_RX,UA);
                    clearAlarm();
                    printf("Successfully connected!\n");
                    break;
                }

            // Cancel the procedure, maximum number of retransmissions exceeded
            clearAlarm();
            printf("Maximum number of retransmissions exceeded!\n");
            return -1;
         }
        break;

    default:
        return -1;
        break;
    }
    
    return dl_identifier;
}


int prepare_frame(char *f_buf, const unsigned char *buf, int bufSize) {
    f_buf[0] = FLAG;
    f_buf[1] = A_TX;
    f_buf[2] = INF_(frame_to_send);
    f_buf[3] = f_buf[1] ^ f_buf[2];

    int num_bytes = 4;
    char bcc2 = 0;

    // Data packet
    for (int i = 0; i < bufSize; i++, num_bytes++) {

        if (buf[i] == FLAG || buf[i] == ESC) {
            // Data byte needs to be stuffed
            f_buf[num_bytes++] = ESC;
            f_buf[num_bytes] = buf[i] ^ 0x20;
        } else {
            f_buf[num_bytes] = buf[i];
        }

        bcc2 ^= buf[i];
    }

    if (bcc2 == FLAG || bcc2 == ESC) {
        // BCC2 Needs to be stuffed
        f_buf[num_bytes++] = ESC;
        f_buf[num_bytes++] = bcc2 ^ 0x20;
    } else {
        f_buf[num_bytes++] = bcc2;
    }

    f_buf[num_bytes++] = FLAG;

    return num_bytes;
}

int waitWriteResponse() {
    state_t state = START_RCV;
    unsigned char c = 0;

    while (state != STOP_RCV) {
        unsigned char byte;

        if ( readByte(&byte) == 1 ) {
            // printf("Read byte = 0x%02X\n", byte);
            
            switch (state) {
                
                case START_RCV:
                    if (byte == FLAG) 
                        state = FLAG_OK;
                    break;
                    
                case FLAG_OK:
                    if (byte == A_TX)
                        state = A_OK;
                    else if (byte != FLAG)
                        state = START_RCV;
                    break;
                    
                case A_OK:
                    if (byte == FLAG)
                        state = FLAG_OK;
                    else if ( (byte == REJ(frame_to_send)) | (byte == RR(NEXT_FRAME(frame_to_send))) ) {
                        c = byte;
                        state = C_OK;
                    } else 
                        state = START_RCV;
                    break;
                    
                case C_OK:
                    if (byte == FLAG)
                        state = FLAG_OK;
                    else if (byte == (A_TX ^ c))
                        state = BCC_OK;
                    else 
                        state = START_RCV;
                    break;
                    
                case BCC_OK:
                    if (byte == FLAG)
                        state = STOP_RCV;
                    else 
                        state = START_RCV;
                    break;

                default:
                    printf("ERROR: Wrong state (%d) in 'receiving supervision frame' state machine", state);
                    break;
            }
        }

        if (alarmEnabled == FALSE) {
            // Timeout
            stats.retransmissions++;
            return -1;
        }
    }

    if (c == RR(NEXT_FRAME(frame_to_send)))
        return 0;
    return -1;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{

    // for (int i = 0; i < bufSize; i++)
    //     printf("byte written: 0x%02X\n",buf[i]);
    // Frame buffer, data bytes (bufSize) + 4 other field bytes + 2 flags
    // If all bytes are stuffed then number of bytes is doubled with the exception of the start and end flags
    unsigned char f_buf[2 *(bufSize + 4) + 2];

    int frameSize = prepare_frame(f_buf, buf, bufSize);

    while (alarmCount < connectionParams.nRetransmissions) {
        // Send frame
        if (writeBytes(f_buf, frameSize) != frameSize) {
            printf("ERROR: writeBytes() didn't write all bytes\n");
            continue; // so para quando escrever
        }
        enableAlarm(connectionParams.timeout);

        // Wait for response
        if (waitWriteResponse() == 0) {
            // Frame successfully akcnowledged
            clearAlarm();
            stats.frames++;
            frame_to_send = NEXT_FRAME(frame_to_send);
            return bufSize;
        } else {
            // Frame rejected or timeout
            alarm(0);
            stats.retransmissions++;
        }
    }

    printf("Maximum number of retransmissions exceeded!\n");
    clearAlarm();
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char byte,c_byte;
    state_t state = START_RCV;
    int char_read = 0;
    unsigned char escape_next=0;
    unsigned char bcc = 0;

    while (state!= STOP_RCV){

        if (readByte(&byte) == 1) {
        
            switch (state)
            {
            case START_RCV:
                if (byte == FLAG)
                    state = FLAG_OK;
                break;

            case FLAG_OK:

                if (byte == A_TX){
                    state = A_OK;
                    }
                else if (byte !=FLAG)
                    state = START_RCV;
                break;

            case A_OK:

                if (byte == INF_0 || byte == INF_1){
                    c_byte = byte;
                    byte = byte >> 7;
                    if (byte == frame_expected){
                    state = C_OK;
                    }
                    else{
                        if(frame_expected) {
                            sendSupervision(A_TX, RR1);
                            state = START_RCV;
                        }
                    else{
                        sendSupervision (A_TX,RR0);
                        state = START_RCV;
                        }
                    }
                }
                else if (byte == FLAG)
                    state = FLAG_OK;
                else 
                    state = START_RCV;
                break;
        
            case C_OK:
                    // printf("c ok\n");

                if (byte == (A_TX ^ c_byte)){
                    state = BCC_OK;
                }
                else if (byte == FLAG)
                    state = FLAG_OK;
                else 
                    state = START_RCV;
                break;
            
            case BCC_OK:
                    // printf("bcc ok\n");

                if (byte == FLAG) {
                    //verificar bbc2, criar funcao
                    int i = char_read - 1;
                    unsigned char bcc2 = packet[i--];
                    
                    while (i>=0){
                        bcc ^= packet[i--];

                    }
                    if (bcc == bcc2){

                        frame_expected ^= 0x01; //want to receive next packet
                        sendSupervision(A_TX, RR(frame_expected));

                        state = STOP_RCV;
                    }
                    else{
                        if(frame_expected) {
                            sendSupervision(A_TX, REJ1);
                        }
                            
                    else {
                            sendSupervision (A_TX,REJ0);
                        }
                        char_read = 0;
                        bcc = 0;
                        state = START_RCV;
                    }

                } else if (byte == ESC) {
                    escape_next = 1;
                } else {
                    if (escape_next) {
                        byte ^= 0x20;
                        escape_next = 0;
                    }
                    packet[char_read++] = byte;
                }
                break;

            default:
                printf("Wrong state");
                break;
            }
        }
    }
        

     return char_read;
}


int waitDiscResponse() {
    state_t state = START_RCV;
    char c = 0;
    char a = 0;

    while (state != STOP_RCV) {
        char byte;

        if ( readByte(&byte) == 1 ) {
            switch (state) {
                
                case START_RCV:
                    if (byte == FLAG) 
                        state = FLAG_OK;
                    break;
                    
                case FLAG_OK:
                    if (byte == A_TX) {
                        a = byte;
                        state = A_OK;
                    } else if (byte == A_RX) {
                        a = byte;
                        state = A_OK_RX;
                    } else if (byte != FLAG)
                        state = START_RCV;
                    break;
                    
                case A_OK:
                    if (byte == FLAG)
                        state = FLAG_OK;
                    else if (byte == DISC) {
                        c = byte;
                        state = C_OK;
                    } else 
                        state = START_RCV;
                    break;

                case A_OK_RX:
                    if (byte == FLAG)
                        state = FLAG_OK;
                    else if (byte == UA) {
                        c = byte;
                        state = C_OK;
                    } else 
                        state = START_RCV;
                    break;
                    
                case C_OK:
                    if (byte == FLAG)
                        state = FLAG_OK;
                    else if (byte == (a ^ c))
                        state = BCC_OK;
                    else 
                        state = START_RCV;
                    break;
                    
                case BCC_OK:
                    if (byte == FLAG)
                        state = STOP_RCV;
                    else 
                        state = START_RCV;
                    break;

                default:
                    printf("ERROR: Wrong state (%d) in 'receiving supervision frame' state machine", state);
                    break;
            }
        }

        if (alarmEnabled == FALSE) {
            // Timeout
            stats.retransmissions++;
            printf("Timeout %d", alarmCount);
            return -1;
        }
    }

    if (c == UA)
        return 0;
    return -1;
}




////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    if (connectionParams.role == LlTx) {

        while (TRUE) {

            // Send DISC frame
            sendSupervision(A_TX, DISC);
            enableAlarm(connectionParams.timeout);

            // Successfully receives DISC
            if (receiveSupervision(A_RX, DISC, TRUE) == 0) {
                clearAlarm();
                sendSupervision(A_RX, UA);
                printf("Successfully disconnected!\n");
                break;
            }

            // Cancel the procedure, maximum number of retransmissions exceeded
            if (alarmCount >= connectionParams.nRetransmissions) {
                clearAlarm();
                printf("Maximum number of retransmissions exceeded!\n");
                return -1;
            }

        }
        
    } else if (connectionParams.role == LlRx) {

        receiveSupervision(A_TX, DISC, FALSE);

        while (TRUE) {

            // Send DISC frame
            sendSupervision(A_RX, DISC);
            enableAlarm(connectionParams.timeout);

            if (waitDiscResponse() == 0) {
                // Successfully receives UA
                clearAlarm();
                printf("Successfully disconnected!\n");
                break;
            } else {
                // Timeout or DISC received
                alarm(0);
                stats.retransmissions++; // ta em dois sitios
            }

            // Cancel the procedure, maximum number of retransmissions exceeded
            if (alarmCount >= connectionParams.nRetransmissions) {
                clearAlarm();
                printf("Maximum number of retransmissions exceeded!\n");

                closeSerialPort(); // deve fechar

                return -1;
            }
        }

    }

    int clstat = closeSerialPort();

    if (showStatistics == TRUE)
        printStatistics();

    return clstat;
}
