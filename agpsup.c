#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

static char string[256];
static int readtonull(int fd)
{
    int l;
    char *b = string;
    for (;;) {
        l = read(fd, b, 1);
        if (l <= 0)
            continue;
        if (!*b)
            break;
	printf( "%c", *b );
    }
    printf( "\n" );
    return 0;
}

static unsigned char ephbuf[256 * 1024];
static const unsigned char setagps[8] = { 0xa0, 0xa1, 0x00, 0x01, 0x35, 0x35, 0x0d, 0x0a };
static const unsigned char agpsresp[9] = { 0xa0, 0xa1, 0x00, 0x02, 0x83, 0x35, 0xb6, 0x0d, 0x0a };
static const unsigned char agpsena[9] = { 0xa0, 0xa1, 0x00, 0x02, 0x33, 0x01, 0x32, 0x0d, 0x0a };

int main(int argc, char *argv[])
{
    int fd;
    char gpsdev[64] = "/dev/rfcomm0";
    //    char gpsdev[64] = "/dev/ttyUSB0";
    unsigned i;

    if( argc > 1 )
        strcpy( gpsdev, argv[1] );

    fd = open(gpsdev, O_RDWR);
    if (fd < 0)
        return -10;

    struct termios tio;
    if ((tcgetattr(fd, &tio)) == -1)
        return -1;
    cfmakeraw(&tio);
    if ((tcsetattr(fd, TCSAFLUSH, &tio)) == -1)
        return -1;
    // add: find baud rate

    unsigned char *ephdata;
    long ephbytes;

    ephdata = ephbuf;
    int ofd = open("Eph.dat", O_RDONLY);
    if (ofd < 0)
        return -2;
    ephbytes = read(ofd, ephbuf, 256 * 1024);
    if (ephbytes < 65536)
        return -3;

    // checksum
    unsigned char csuma, csumb = 0;
    for (i = 0; i < 0x10000; i++)
        csumb += ephdata[i];
    csuma = csumb;
    for (; i < ephbytes; i++)
        csuma += ephdata[i];

   // AGPS download startup: send command, get ack - maybe put in loop?
    do {
	printf( "Startup\n" );
        do { // flush input buffer
            i = read(fd, string, 256);
        } while( i == 256 );
        write(fd, setagps, 8);
        i = 0;
	string[0] = 0;
	int cnt=0;
        while ( i < 0 || string[0] != '\xa0') {
            i = read(fd, string, 1);
	    cnt += i;
	    if( cnt > 5000 )
		continue;
	}
	printf( "Response\n" );
        while (i < 256) {
            i += read(fd, &string[i], 256 - i); // read extra junk after the response, sometimes partial sentence queued and going out.
            if (i > 8 && string[8] == 0x0a)
                break;
        }
    } while( memcmp(&string[i - 9], agpsresp, 9) );
    printf( "Venus Ready\n" );
    
    usleep(500000);  // wait for switch into AGPS mode - this is required.
    /* start the transmission */
    sprintf(string, "BINSIZE = %ld Checksum = %d Checksumb = %d ", ephbytes, csuma, csumb);
    write(fd, string, strlen(string) + 1);
    printf("%s\n", string);
    readtonull(fd);

#define BLKSIZ 8192
    unsigned tot = ephbytes;
    while (ephbytes > 0) {
        printf("%ld%% ", (tot - ephbytes) * 100 / tot);

        write(fd, ephdata, ephbytes > BLKSIZ ? BLKSIZ : ephbytes);
	readtonull(fd);        // OK or Error, null terminated
        ephbytes -= BLKSIZ;
        ephdata += BLKSIZ;
    }
    // Status "END" or "Error2"
    readtonull(fd);            // END
    //    readtonull(fd);            // END
    printf( "Finish and emable\n");
    sleep(1);
    write(fd, agpsena, 9);
    // maybe get ack?
    sleep(1);
    close(fd);
    return 0;
}
