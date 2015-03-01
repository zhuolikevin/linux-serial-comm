#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <stdint.h>
#define GET_ARRAY_LEN(array,len){len = (sizeof(array) / sizeof(array[0]));}

#define MESSAGE_MAX_LEN         256

#define OBJ_ModuleInfo              6  // Get Module Info
#define OBJ_ChanPort              170  // Channel Switch Port
#define OBJ_ChanAtten             171  // Channel Attenuation

char mod_info[MESSAGE_MAX_LEN];

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
} Command;

typedef enum {
    RX_ESCCHAR  = 0xdd,         /* only recognized on rs232 */
    RX_ESCSTART = 0x01,
    RX_ESCEND   = 0x02,
    RX_REREAD   = 0x03,
    RX_ESCAPED  = 0x04,
} SerialRX;

short swapBE16(short src)
{
    short dst;
    char *srcp = (char *)&src;
    char *dstp = (char *)&dst;
    dstp[1] = srcp[0]; // MSB
    dstp[0] = srcp[1]; // LSB
    return dst;
}

 unsigned char cmd_pool[4][13] = {{0xdd, 0x01, 0x01, 0x05, 0x02, 0x06, 0x01, 0x00, 0x01, 0xdd, 0x02},   // modinfo
	                   			       {0xdd, 0x01, 0x01, 0x05, 0x02, 0x96, 0x03, 0x00, 0x93, 0xdd, 0x02},  // readtemp
                                       {0xdd, 0x01, 0x02, 0x07, 0x01, 0xaa, 0x0c, 0x01, 0x00, 0x02, 0xa1, 0xdd, 0x02},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
/*									  
const int cmd_pool[2][11] = {{221, 1, 1, 5, 2, 6, 1, 0, 1, 221, 2},   // modinfo
	                   		 {221, 1, 1, 5, 2, 150, 3, 0, 147, 221, 2}};  // readtemp
*/

int cmd_num = 2;

int len;

int last_mid;

int formming_cmd = 0;

unsigned char *cmd;

unsigned char buffer[MESSAGE_MAX_LEN*2 + 4];

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
    	//printf("serial port opened successfully as fd :%d\n", fd);
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
	int cmd_len; 
	GET_ARRAY_LEN(cmd_pool[cmd_num], cmd_len);
	int i;
	if (!formming_cmd){
		for (i=0; i<cmd_len; i++){
            printf("%d ",cmd_len);
			n_written += write(fd, &(cmd_pool[cmd_num][i]), 1);
			
		}
	}else{
		for (i=0; i<len; i++){
			printf("%d ",len);
			n_written += write(fd, &(cmd_pool[3][i]), 1);
			//n_written += write(fd, &(cmd_pool[cmd_num][i]), 1);
		}

	}
	//n_written = write(fd, &(cmd_pool[cmd_num]), cmd_len);
	printf("bytes written :%d\n",(int)n_written);
	
	if ((int)n_written > 0)
		printf("Serial port writing succeeded\n");
	else
		printf("Serial port writing failed\n");

}

/* read response from the port */
static void read_port(fd){
    
	printf("zenmebujinlai");
	unsigned char response[1024];
	int n_read=0;
	
    int i=0;
	int getend = 0;
    
	printf("haha");
	while (!getend){
		printf("ssss\n");
		n_read += read(fd, &response[i], 1);
		printf("%x ",response[i]);
		if (response[i] == 0x02 && response[i-1] == 0xdd)   // reach the end of the response
			getend = 1;
		i++;
	}
	printf("haha2");

	if (n_read > 0){
		printf("bytes read: %d\n", (int)n_read);
		printf("read successfully\n");
		int i;
		for (i = 0; i<n_read; i++){
			printf("%d  ", (int)response[i]);
		}
	}
	else{
		printf("read failed\n");
	}
	

}

unsigned char checksum(unsigned char *buffer, int len) 
{
    unsigned char xor = 0;

    while(len--)
        xor ^= *(buffer++);

    return xor;
}

static unsigned char *serial_encode(unsigned char *data, int datalen){

	/*
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
    */
	int buflen = 0;
    int i;

    // copy message data into internal buffer, with framing and escaped chars

    cmd_pool[3][buflen++] = RX_ESCCHAR;
    cmd_pool[3][buflen++] = RX_ESCSTART;
    for(i = 0; i < datalen; i++) {
        if(data[i] == RX_ESCCHAR) {
            cmd_pool[3][buflen++] = RX_ESCCHAR;
            cmd_pool[3][buflen++] = RX_ESCAPED;
        } else
            cmd_pool[3][buflen++] = data[i];
    }
    cmd_pool[3][buflen++] = RX_ESCCHAR;
    cmd_pool[3][buflen++] = RX_ESCEND;

	len = buflen;

	//return buffer;
}

static unsigned char *translate_cmd(int cmd, int obj, int inst, int parm, int *data, int datalen){

	unsigned char sendbuffer[MESSAGE_MAX_LEN];
	unsigned char *final_buffer;
    int sendbuflen = MESSAGE_MAX_LEN;
    unsigned char mid;
	
	last_mid = (last_mid + 1) % 256;
    if(last_mid == 0) last_mid = 1;

	sendbuffer[0] = 2;
	sendbuffer[1] = datalen + 5;
	sendbuffer[2] = cmd;
	sendbuffer[3] = obj;
	sendbuffer[4] = inst;
	sendbuffer[5] = parm;
	memcpy(&sendbuffer[6], data, datalen);
	sendbuffer[6+datalen] = checksum(sendbuffer, 6+datalen);
	len = 7 + datalen;
    
 	final_buffer = serial_encode(sendbuffer, len);

	//return final_buffer;
}


		

void main(int argc, char *argv[])
{
	int fd;
		
	char *dev = "/dev/ttyUSB0";
    
	// open the serial port
	fd = open_port(dev);

    // set speed to 115,200 bps, 8n1 (no parity)
	set_interface_attribs (fd, B115200, 0);     
	

    if(!strcmp(argv[1],"modinfo")){
		cmd_num = 0;
	}
	else if (!strcmp(argv[1], "temp")){
		cmd_num = 1;
	}
	else if(!strncmp(argv[1],"channel",7)) {

		int switch_num, channel_num, port_num;

		switch_num = argv[1][8]-'0';

		channel_num = (argv[1][10]-'0')*10 + (argv[1][11]-'0');

		port_num = argv[1][13]-'0';
		
		short val = swapBE16(port_num);

		//cmd = translate_cmd(CMD_WRITE, OBJ_ChanPort, channel_num, 1, &val, sizeof(short));
        translate_cmd(CMD_WRITE, OBJ_ChanPort, channel_num, 1, &val, sizeof(short));

		//cmd = translate_cmd(CMD_READ, OBJ_ModuleInfo,1, 0, 0, 0);
		/*
      	printf("cmdlen: %d\n", len);
		int i;
		for (i=0; i<len; i++){
			printf("%d ",cmd[i]);
		}
		printf("\n");
        for (i=0;i<13;i++){
			printf("%d ", cmd_pool[2][i]);
		}
		*/

		formming_cmd = 1;

	}
	else if (!strcmp(argv[1], "switch")){
		cmd_num = 2;
	}

	// write command to the serial port
	write_port(fd);
	 printf("kule");

	printf("write finished\n");
   	// read response from the serial port
	read_port(fd);
	printf("read finished\n");

	// close the serial port
	close(fd);
}