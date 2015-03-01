/*
 *  Program name: serial
 *  Platform: GNU/Linux
 *  Author: Zhuo Li
 *  Late Modification: March, 1st, 2015
 *  Brief Description: This program can realize the communication with NISTICA WSS board 
 *                     via serial port under Linux environment. 
 *  Copyright: Lightwave Research Laboratory @ Columbia Unviersity
 */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>

#define MESSAGE_MAX_LEN         256
#define OBJ_ModuleInfo              6  // Get Module Info
#define OBJ_ChanPort              170  // Channel Switch Port
#define OBJ_ChanAtten             171  // Channel Attenuation

/* Global Variables */
char mod_info[MESSAGE_MAX_LEN];             // Store module information response
int len;                                    // Store sending serial frame length
int last_mid;                               // Store MID
unsigned char *cmd;                         // Store serial frame
unsigned char buffer[MESSAGE_MAX_LEN*2 + 4];// Buffer for formming the serial frame

/* Command Table */
typedef enum {
    CMD_NOOP      = 0,
    CMD_WRITE     = 1,
    CMD_READ      = 2,
    CMD_ABORT     = 3,
    CMD_VERIFY    = 4,
    CMD_SWITCH    = 5,
    CMD_COMMIT    = 6,
    CMD_START     = 7,
    CMD_GETRESULT = 8,
	CMD_TABLEADD  = 32,        // for switch choosing
} Command;

/* RS-232 serial communication framing key words */
typedef enum {
    RX_ESCCHAR  = 0xdd,         
    RX_ESCSTART = 0x01,
    RX_ESCEND   = 0x02,
    RX_REREAD   = 0x03,
    RX_ESCAPED  = 0x04,
} SerialRX;

/* 
 * converting to big-endian
 * module uses big-endian (most-significant byte first) for communications, so
 * we need to swap if host is little-endian like a PC.
 */
short swapBE16(short src)
{
    short dst;
    char *srcp = (char *)&src;
    char *dstp = (char *)&dst;
    dstp[1] = srcp[0]; // MSB
    dstp[0] = srcp[1]; // LSB
    return dst;
}

/* Generating random MID */
int randomStartId(void)
{
	time_t t = time(NULL);
    return (((int)t) % 255) + 1;
}

/* for baud rate and parity configuration */
static int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                perror("set_interface_attribs: error from tcgetattr");
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                perror("set_interface_attribs: error from tcsetattr");
                return -1;
        }
        return 0;
}

/* open the serial port */
static int open_port(char *dev)
{
   	
  	int fd; // File descriptor for the port 

  	fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
  	if (fd == -1)
  	{
    	perror("open_port: Unable to open this device");
  	}
  	else{
    	fcntl(fd, F_SETFL, 0);
  	}

	last_mid = randomStartId();

    return (fd);
}

/* close the serial port */
static void close_port(fd){
	close(fd);
}

/* write command to the port */
static void write_port(fd){
	
	int n_written = 0;
	int i;

	printf("Writing command...\n");

	for (i=0; i<len; i++){
		n_written += write(fd, &(cmd[i]), 1);
	}
		
	if ((int)n_written > 0){
		printf("[Serial Port Write Successfully] ");
		printf("Total bytes written :%d\n",(int)n_written);
	}else{
		printf("[Serial Port Writing Failed]\n");
	}
}

/* read response from the port */
static void read_port(fd){
    
	unsigned char response[1024];
	int n_read=0;
	
    int i=0;
	int getend = 0;
    
	printf("Reading response...\n");

	while (!getend){
		n_read += read(fd, &response[i], 1);
		// if reach the end of the response
		if (response[i] == 0x02 && response[i-1] == 0xdd) 
			getend = 1;
		i++;
	}

	if (n_read > 0){
		printf("[Serial Port Read Successfully] ");
		printf("Total bytes read: %d\n", (int)n_read);
		printf("response message:\n");
		int i;
		for (i = 0; i<n_read; i++){
			printf("%x  ", response[i]);
		}
		printf("\n");
	}
	else{
		printf("[Serial Port Reading Failed]\n");
	}
}

/* calculating the checksum */
static unsigned char checksum(unsigned char *buffer, int len) 
{
    unsigned char xor = 0;

    while(len--)
        xor ^= *(buffer++);

    return xor;
}

/* forming the serial communication frame */
static unsigned char *serial_encode(unsigned char *data, int datalen){

	
    int buflen = 0;
    int i;

    // copy message data into internal buffer, with framing and escaped chars

    buffer[buflen++] = RX_ESCCHAR;
    buffer[buflen++] = RX_ESCSTART;
    for(i = 0; i < datalen; i++) {
        if(data[i] == RX_ESCCHAR) {
            buffer[buflen++] = RX_ESCCHAR;
            buffer[buflen++] = RX_ESCAPED;
        } else
            buffer[buflen++] = data[i];
    }
    buffer[buflen++] = RX_ESCCHAR;
    buffer[buflen++] = RX_ESCEND;

	len = buflen;
    
	return buffer;
}

/* translate user command to serial command */
static unsigned char *translate_cmd(int cmd, int obj, int inst, int parm, short *data, int datalen, unsigned char table){

	unsigned char sendbuffer[MESSAGE_MAX_LEN];
	unsigned char *final_buffer;
    int sendbuflen = MESSAGE_MAX_LEN;
    unsigned char mid;
	
	last_mid = (last_mid + 1) % 256;
    if (last_mid == 0) last_mid = 1;
    
	if (table == 0xff){                     // WSS not selected, default to be WSS A
		sendbuffer[0] = last_mid;
		sendbuffer[1] = datalen + 5;
		sendbuffer[2] = cmd;
		sendbuffer[3] = obj;
		sendbuffer[4] = inst;
		sendbuffer[5] = parm;
		memcpy(&sendbuffer[6], data, datalen);
		sendbuffer[6+datalen] = checksum(sendbuffer, 6+datalen);
		len = 7 + datalen;
	}else{                                  // WSS selected
		sendbuffer[0] = last_mid;
		sendbuffer[1] = datalen + 6;
		sendbuffer[2] = cmd;
		sendbuffer[3] = obj;
		sendbuffer[4] = table;
		sendbuffer[5] = inst;
		sendbuffer[6] = parm;
		memcpy(&sendbuffer[7], data, datalen);
		sendbuffer[7+datalen] = checksum(sendbuffer, 7+datalen);
		len = 8 + datalen;
	}
    
 	final_buffer = serial_encode(sendbuffer, len);

	return final_buffer;
}

/* program beginning */
void main(int argc, char *argv[])
{
	/* determine the command */
    if(!strcmp(argv[1],"modinfo")){
        
		// get module information
		cmd = translate_cmd(CMD_READ, OBJ_ModuleInfo,1, 0, 0, 0, 0xff);
	
	}
	else if(!strncmp(argv[1],"channel",7)) {
        // change channel ports
		// [example] channel.1.06.2
		// switch the 06 channel in WSS B to port 2
		// detail can be found in Readme.txt
		int switch_num, channel_num, port_num;

		switch_num = argv[1][8]-'0';
		channel_num = (argv[1][10]-'0')*10 + (argv[1][11]-'0');
		port_num = argv[1][13]-'0';
		
		if (port_num > 2){
			printf("<Port ID> should be in the range of 0 ~ 2\n");
			exit(0);
		}

        short table;
		int command;

		if (switch_num == 0){
			table = 0xff;
			command = CMD_WRITE;
		}else if (switch_num == 1){
			table = 0x01;
			command = CMD_WRITE + CMD_TABLEADD;
		}else{
			printf("<WSS ID> should only be 0 or 1\n");
			exit(0);
		}
		short val = swapBE16(port_num);
   
		cmd = translate_cmd(command, OBJ_ChanPort, channel_num, 1, &val, sizeof(short), table);
	}
	else if(!strncmp(argv[1],"attenua",7)) {
        // channel attenuation
		// [example] attenua.1.06.20
		// attenuate the 06 channel in WSS B by 2dB
		// detail can be found in Readme.txt
		int switch_num, channel_num, att_num;

		switch_num = argv[1][8]-'0';
		channel_num = (argv[1][10]-'0')*10 + (argv[1][11]-'0');
		att_num = (argv[1][13]-'0')*10 + (argv[1][14]-'0');

		short table;
		int command;

		if (switch_num == 0){
			table = 0xff;
			command = CMD_WRITE;
		}else if (switch_num == 1){
			table = 0x01;
			command = CMD_WRITE + CMD_TABLEADD;
		}else{
			printf("<WSS ID> should only be 0 or 1\n");
			exit(0);
		}

		short val = swapBE16(att_num);

		cmd = translate_cmd(command, OBJ_ChanAtten, channel_num, 1, &val, sizeof(short), table);
	}
	else{

		printf("Running format:\n \
    >> serial <command> [option]\n \
	current support <command>:\n \
	1. modinfo\n \
	2. channel.<WSS ID>.<channel ID>.<Port ID>\n \
	3. attenua.<WSS ID>.<channel ID>.<Attenuation>\n");
		exit(0);
	}

	int fd;		
	char *dev = "/dev/ttyUSB1";    
	/* open the serial port */
	fd = open_port(dev);
    /* set speed to 115,200 bps, 8n1 (no parity) */
	set_interface_attribs (fd, B115200, 0); // Baud Rate must be 115200 for NISTICA
	/* write command to the serial port */
	write_port(fd);
   	/* read response from the serial port */
	read_port(fd);
	/* close the serial port */
	close(fd);

}
