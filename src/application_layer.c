// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <stdio.h>

#define PCKT_C_START 0x1
#define PCKT_C_DATA 0x2
#define PCKT_C_END 0x3
#define PCKT_T_FILE_SZ 0x0
#define PCKT_T_FILE_NM 0x1

#define FRAGMENT_SZ 3000
#define DATA_PCKT_SZ (FRAGMENT_SZ + 4)
#define PCKT_L2 (FRAGMENT_SZ / 256)
#define PCKT_L1 (FRAGMENT_SZ % 256)




void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout
    };
    strncpy(connectionParameters.serialPort, serialPort, sizeof(connectionParameters.serialPort) - 1);

    // /* ----------TEST-------- */
    // // RECEIVER --------------------
    // if (!strcmp(role, "rx")){
    //     connectionParameters.role = LlRx;
    //     llopen(connectionParameters);
    //     printf("Connection open\n");
    //     unsigned char packet[5] = {0};
    //     llread(packet);
    //     llclose(TRUE);
    // }
    // // TRANSMITTER -----------------
    // else if (!strcmp(role, "tx")){
    //     connectionParameters.role = LlTx;
    //     llopen(connectionParameters);
    //     printf("Connection open\n");
    //     unsigned char buf[4]; 
    //     buf[0] = 0x7e;
    //     buf[1] = 0xf1;
    //     buf[2] = 0x7d;
    //     buf[3] = 0xab;
    //     llwrite(buf, 4);
    //     llclose(TRUE);
    // }

    // RECEIVER --------------------
    if (!strcmp(role, "rx")){
        connectionParameters.role = LlRx;

        llopen(connectionParameters);
                //sleep(1);


        unsigned char start_pckt[256] = {0};

        int file_sz = 0;
        unsigned char filename_rcvd[256] = {0};

        while (TRUE){
            if (llread(start_pckt) < 1){
                continue;
            }
            
            if (start_pckt[0] == PCKT_C_START){
                if (start_pckt[1] == PCKT_T_FILE_SZ){
                    int filename_size_sz = start_pckt[2];
                    file_sz = start_pckt[2];
                    //memcpy(&file_sz, &start_pckt[1], filename_size_sz);
                    if (start_pckt[3 + filename_size_sz] == PCKT_T_FILE_NM){
                        memcpy(filename_rcvd, &start_pckt[5 + filename_size_sz], start_pckt[4 + filename_size_sz]);
                    }
                    // for (int i = 0; i< 9 + start_pckt[4 + filename_size_sz]; i++)
                        // printf("START PCKT of size %d = 0x%02X\n", 9+file_sz, start_pckt[i]);
                }
                break;
            }
        }


        int fd_target = open(filename_rcvd, O_WRONLY | O_CREAT, S_IRUSR);
        int bytes_read = 0;
        unsigned char data_pckt[DATA_PCKT_SZ] = {0};
        unsigned char seq_n = -1;
        unsigned char l2, l1 = 0;
        // printf("RCVNG data packet -------\n"); 
        while (TRUE){
        //sleep(1);
            llread(data_pckt);

            if ((data_pckt[0] == PCKT_C_DATA)){
                l2 = data_pckt[2];
                l1 = data_pckt[3];
                bytes_read += write(fd_target, &data_pckt[4], l2*256+l1);
                // printf("bytes read %d\n", bytes_read);
                // printf("fd %d\n", fd_target);
            }
            else if (data_pckt[0] == PCKT_C_END)
            {
                // printf("end packet received\n");
                break;
            }
            
        }
            // printf("End of data packet -------\n"); 


        close(fd_target);
        llclose(TRUE);

    }
    // TRANSMITTER -----------------
    else if (!strcmp(role, "tx")){
        connectionParameters.role = LlTx;
        int fd_data = open(filename, O_RDONLY);

        llopen(connectionParameters);

        struct stat st;
        stat(filename, &st);
        int file_sz = st.st_size;
        int filename_sz = strlen(filename);

        // START PACKET ASSEMBLY
        unsigned char ctrl_pckt[256] = {0};
        ctrl_pckt[0] = PCKT_C_START;
        ctrl_pckt[1] = PCKT_T_FILE_SZ;
        ctrl_pckt[2] = sizeof(int);
        memcpy(&ctrl_pckt[3], &file_sz, sizeof(int));
        ctrl_pckt[7] = PCKT_T_FILE_NM;
        ctrl_pckt[8] = filename_sz;
        //printf("file name %s\n", filename);
        memcpy(&ctrl_pckt[9], filename, filename_sz);

        //printf("Sending start packet -------\n"); 
        if (!llwrite(ctrl_pckt, 5 + sizeof(int) + filename_sz)){
            return;
        }
        //sleep(1);
        //for (int i = 0; i< 5 + sizeof(int) + filename_sz; i++)
        //    printf("SRT PCKT = 0x%02X\n", ctrl_pckt[i]);
        
        //printf("Start packet sent    -------\n"); 

        // DATA PACKETS ASSEMBLY
        int n_fragments = (file_sz + FRAGMENT_SZ - 1) / FRAGMENT_SZ;
        // printf("n fragements %d\n", n_fragments);
        unsigned char data_pckt[DATA_PCKT_SZ] = {0};
        int n = 0;
        while (n < n_fragments){
            printf("n %d\n", n);
            memset(data_pckt, 0, DATA_PCKT_SZ);
            data_pckt[0] = PCKT_C_DATA;
            int fragment_sz = read(fd_data, &data_pckt[4], FRAGMENT_SZ);
            // printf("fragment read : %d\n", fragment_sz);
            data_pckt[1] = n % 100;
            data_pckt[2] = fragment_sz / 256;
            data_pckt[3] = fragment_sz % 256;

            n++;
            if (!fragment_sz)
                break;
            // printf("DATA packet start -----\n");
            llwrite(data_pckt,4+fragment_sz);
            // printf("DATA packet end -----\n");
            
        }

        //sleep(1);

        // END PACKET ASSEMBLY
        ctrl_pckt[0] = PCKT_C_END;
        ctrl_pckt[1] = PCKT_T_FILE_SZ;
        ctrl_pckt[2] = sizeof(int);
        memcpy(&ctrl_pckt[3], &file_sz, sizeof(int));
        ctrl_pckt[7] = PCKT_T_FILE_NM;
        ctrl_pckt[8] = filename_sz;
        memcpy(&ctrl_pckt[9], &filename, filename_sz);
        // printf("Sending end packet -------\n"); 
        if (!llwrite(ctrl_pckt, 5 + sizeof(int) + filename_sz)){
            return;
        }
        // printf("ENd packet sent    -------\n"); 

        close(fd_data);
        llclose(TRUE);

    }
}