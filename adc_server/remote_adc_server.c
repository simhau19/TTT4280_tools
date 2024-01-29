/*
remote_adc_server.c.c
Public Domain
January 2024 - Server functionality added by Simen Haug
January 2018, Kristoffer KjÃƒÂ¦rnes & Asgeir BjÃƒÂ¸rgan
Based on example code from the pigpio library by Joan @ raspi forum and github
https://github.com/joan2937 | http://abyz.me.uk/rpi/pigpio/

Compile and run with make:
sudo make run

or use:
gcc -Wall -g remote_adc_server.c -o remote_adc_server -lpthread -lpigpio -lm
...and run with:
sudo ./remote_adc_server

This code bit bangs SPI on several devices using DMA.

Using DMA to bit bang allows for two advantages
1) the time of the SPI transaction can be guaranteed to within a
   microsecond or so.

2) multiple devices of the same type can be read or written
   simultaneously.

This code reads several MCP3201 ADCs in parallel, and sends the data over TCP.
Each MCP3201 shares the SPI clock and slave select lines but has
a unique MISO line. The MOSI line is not in use, since the MCP3201 is single
channel ADC without need for any input to initiate sampling.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>

#include <pigpio.h>
#include <math.h>
#include <time.h>

#include <sys/socket.h>
#include <netdb.h> 
#include <netinet/in.h> 
#include <arpa/inet.h>

/////// USER SHOULD MAKE SURE THESE DEFINES CORRESPOND TO THEIR SETUP ///////
#define ADCS 5      // Number of connected MCP3201.

/* RPi PIN ASSIGNMENTS */
#define MISO1 5      // ADC 1 MISO (BCM 4 aka GPIO 4).
#define MISO2 4     //     2
#define MISO3 20    //     3
#define MISO4 23    //     4
#define MISO5 3     //     5

#define MOSI 10     // GPIO for SPI MOSI (BCM 10 aka GPIO 10 aka SPI_MOSI). MOSI not in use here due to single ch. ADCs, but must be defined anyway.
#define SPI_SS 19   // GPIO for slave select (BCM 8 aka GPIO 8 aka SPI_CE0).
#define CLK 18      // GPIO for SPI clock (BCM 11 aka GPIO 11 aka SPI_CLK).
/* END RPi PIN ASSIGNMENTS */

#define BITS 12            // Bits per sample.
#define BX 4               // Bit position of data bit B11. (3 first are t_sample + null bit)
#define B0 (BX + BITS - 1) // Bit position of data bit B0.

#define NUM_SAMPLES_IN_BUFFER 300 // Generally make this buffer as large as possible in order to cope with reschedule.

#define REPEAT_MICROS 32 // Reading every x microseconds. Must be no less than 2xB0 defined above

#define DEFAULT_NUM_SAMPLES 31250 // Default number of samples for printing in the example. Should give 1sec of data at Tp=32us.

int MISO[ADCS]={MISO1, MISO2, MISO3, MISO4, MISO5}; // Must be updated if you change number of ADCs/MISOs above
/////// END USER SHOULD MAKE SURE THESE DEFINES CORRESPOND TO THEIR SETUP ///////


/* NETWORK */
#define SERVER_PORT 4280

/**
 * This function extracts the MISO bits for each ADC and
 * collates them into a reading per ADC.
 *
 * \param adcs Number of attached ADCs
 * \param MISO The GPIO connected to the ADCs data out
 * \param bytes Bytes between readings
 * \param bits Bits per reading
 * \param buf Output buffer
*/
void getReading(int adcs, int *MISO, int OOL, int bytes, int bits, char *buf)
{
   int p = OOL;
   int i, a;

   for (i=0; i < bits; i++) {
      uint32_t level = rawWaveGetOut(p);
      for (a=0; a < adcs; a++) {
         putBitInBytes(i, buf+(bytes*a), level & (1<<MISO[a]));
      }
      p--;
   }
}

void adcRead(uint16_t sample_count, uint16_t channel_count, uint16_t* val, double* sample_period_us){

    // Construct the wave from added data.
    static rawWaveInfo_t rwi;
    static int wid = -1;
    static int botCB, topOOL;
    static float cbs_per_reading;

    if (wid < 0) {
        wid = gpioWaveCreate();

        if (wid < 0){
            fprintf(stderr, "Can't create wave, buffer size %d too large?\n", NUM_SAMPLES_IN_BUFFER);
            exit(1);
        }
        rwi = rawWaveInfo(wid);
        botCB = rwi.botCB;
        topOOL = rwi.topOOL;
        cbs_per_reading = (float)rwi.numCB / (float)NUM_SAMPLES_IN_BUFFER;
    }
    


    // Start DMA engine and start sending ADC reading commands
    gpioWaveTxSend(wid, PI_WAVE_MODE_REPEAT);

    // Read back the samples
    double start_time = time_time();
    int reading = 0;
    int sample = 0;

    while (sample < sample_count) {
        // Get position along DMA control block buffer corresponding to the current output command.
        int cb = rawWaveCB() - botCB;
        int now_reading = (float) cb / cbs_per_reading;

        while ((now_reading != reading) && (sample < sample_count)) {
            // Read samples from DMA input buffer up until the current output command

            // OOL are allocated from the top down. There are BITS bits for each ADC
            // reading and NUM_SAMPLES_IN_BUFFER ADC readings. The readings will be
            // stored in topOOL - 1 to topOOL - (BITS * NUM_SAMPLES_IN_BUFFER).
            // Position of each reading's OOL are calculated relative to the wave's top
            // OOL.
            int reading_address = topOOL - ((reading % NUM_SAMPLES_IN_BUFFER)*BITS) - 1;

            char rx[8];
            getReading(channel_count, MISO, reading_address, 2, BITS, rx);

            // Convert and save to output array
            for (int i=0; i < channel_count; i++) {
                val[sample*channel_count+i] = (rx[i*2]<<4) + (rx[(i*2)+1]>>4);
            }

            ++sample;

            if (++reading >= NUM_SAMPLES_IN_BUFFER) {
                reading = 0;
            }
        }
        usleep(1000);
    }

    double end_time = time_time();

    double nominal_period_us = 1.0*(end_time-start_time)/(1.0*sample_count)*1.0e06;

    *sample_period_us = floor(nominal_period_us); //the clock is accurate only to us resolution


}

// will only return when client is disconnected
void handleClient(int client_fd){
    char* out_buf = NULL;
    size_t out_size = 0;
    uint16_t* val; 
    double* sample_period_us;
    uint16_t error_response[1] = {0};

    //expect 2 uint32 as input
    //first is sample count, second is channel count
    uint32_t in_buf[2]; 
    ssize_t in_size = 0;

    uint32_t sample_count, channel_count = 0;

    while(1){
        in_size = recv(client_fd, in_buf, sizeof(in_buf), 0);

        bool invalid_input = false;
        bool input_changed = false;

        if(in_size == -1){
            perror("ERROR: Receiving from socket failed\n");
            continue;
        }
        if(in_size == 0){
            break; //client disconnected
        }

        if(in_size != sizeof(in_buf)){
            invalid_input = true;
        }

        if (in_buf[0] != sample_count){
            input_changed = true;
            sample_count = in_buf[0];
        }
        if (in_buf[1] != channel_count){
            input_changed = true;
            channel_count = in_buf[1];
        }
        
        if(sample_count == 0 || channel_count == 0){
            invalid_input = true;
        }

        if(invalid_input){
            send(client_fd, error_response, sizeof(error_response), 0);
            printf("ERROR: Received invalid input from client\n");
            continue;
        }

        printf("   Request received, preparing data\n");

        if(input_changed){ 
            //reallocate output buffer
            out_size = sizeof(uint16_t)*sample_count*channel_count + sizeof(double);
            out_buf = realloc((void*)out_buf, out_size);
            if (out_buf == NULL){
                fprintf(stderr, "ERROR: Failed to allocate memory for output buffer. Terminating connection\n");
                break;
            }
            sample_period_us = (double*)out_buf; //First part is the sample period
            val = (uint16_t*) (out_buf + sizeof(double)); //Second part is the adc data

        }


        adcRead(sample_count, channel_count, val, sample_period_us);
        int sent = send(client_fd, out_buf, out_size, 0);
        if (sent != out_size){
            fprintf(stderr, "ERROR: Failed to send data to client. Terminating connection\n");
            break;
        }


    }
    free(out_buf);

}

_Noreturn void server(void)
{
    int listen_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;

    // Create listener socket
    listen_fd = socket(AF_INET, SOCK_STREAM, 0); //IPv4, TCP, default protocol
    if (listen_fd < 0) {
        fprintf(stderr, "ERROR opening socket\n");
        exit(1);
    }

    int reuse = 1;
    if (setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == -1) {
        fprintf(stderr, "ERROR setting socket options\n");
        exit(1);
    }

    // Initialize socket structure
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET; //IPv4
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); //anyone can connect
    server_addr.sin_port = htons(SERVER_PORT);

    // Bind the host address using bind() call
    if (bind(listen_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        fprintf(stderr, "ERROR on binding\n");
        exit(1);
    }

    // Start listening for the clients
    listen(listen_fd,1);
    socklen_t clilen = sizeof(client_addr);

    // Accept actual connection from the client
    while (1){
        printf("Waiting for client to connect...\n");

        client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &clilen);
        if (client_fd < 0) {
            fprintf(stderr, "ERROR on accept\n");
            close(client_fd);
            continue;
        }

        char clientIP_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, clientIP_str, INET_ADDRSTRLEN );
        printf("Accepted connection from %s\n", clientIP_str);
        
        handleClient(client_fd); //Returns when client is disconnected
        printf("Terminated connection with %s\n", clientIP_str);
        close(client_fd);
        
    }

    close(client_fd);
    close(listen_fd);
}


int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    // SPI transfer settings, time resolution 1us (1MHz system clock is used)
    rawSPI_t rawSPI =
    {
       .clk     =  CLK,  // Defined before
       .mosi    =  MOSI, // Defined before
       .ss_pol  =  1,   // Slave select resting level.
       .ss_us   =  1,   // Wait 1 micro after asserting slave select.
       .clk_pol =  0,   // Clock resting level.
       .clk_pha =  0,   // 0 sample on first edge, 1 sample on second edge.
       .clk_us  =  1,   // 2 clocks needed per bit so 500 kbps.
    };

    // Change timer to use PWM clock instead of PCM clock. Default is PCM
    // clock, but playing sound on the system (e.g. espeak at boot) will start
    // sound systems that will take over the PCM timer and make adc_sampler.c
    // sample at far lower samplerates than what we desire.
    // Changing to PWM should fix this problem.
    gpioCfgClock(5, 0, 0);

    // Initialize the pigpio library
    if (gpioInitialise() < 0) { 
       return 1;
    }

    // Set the selected CLK, MOSI and SPI_SS pins as output pins
    gpioSetMode(rawSPI.clk,  PI_OUTPUT);
    gpioSetMode(rawSPI.mosi, PI_OUTPUT);
    gpioSetMode(SPI_SS,      PI_OUTPUT);

    // Flush any old unused wave data.
    gpioWaveAddNew();

    // Construct bit-banged SPI reads. Each ADC reading is stored separatedly
    // along a buffer of DMA commands (control blocks). When the DMA engine
    // reaches the end of the buffer, it restarts on the start of the buffer
    int offset = 0;
    int i;
    char buf[2];
    for (i=0; i < NUM_SAMPLES_IN_BUFFER; i++) {
        buf[0] = 0xC0; // Start bit, single ended, channel 0.

        rawWaveAddSPI(&rawSPI, offset, SPI_SS, buf, 2, BX, B0, B0);
        offset += REPEAT_MICROS;
    }

    // Force the same delay after the last command in the buffer
    gpioPulse_t final[2];
    final[0].gpioOn = 0;
    final[0].gpioOff = 0;
    final[0].usDelay = offset;

    final[1].gpioOn = 0; // Need a dummy to force the final delay.
    final[1].gpioOff = 0;
    final[1].usDelay = 0;

    gpioWaveAddGeneric(2, final);

    server();

    gpioTerminate();

    return 0;
}
