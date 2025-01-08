// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

int main(){
    //----------------//
    //CONECTION//
    //--------------//
    //opens the arduino "file"
    char serialPortName[] = "/dev/ttyUSB0";
    int baudRate = B9600;
    int serial_port = open(serialPortName, O_RDWR);

    // Check if the serial port is open correctly
    if(serial_port < 0){
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    //-----------------//
    //TERMIOS CONFIG
    //---------------//
    //used for configuring the serial port
    struct termios tty;
    //read in existing config
    if(tcgetattr(serial_port, &tty) != 0){
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    
    //control modes(c_cflags)
    //PARENB (Parity)
    tty.c_cflag &= ~PARENB; //most common

    //CSTOPB (Num. Stop Bits)
    //stop bits are used in async data transfers which indicate the end of transfer
    tty.c_cflag &= ~CSTOPB;//most common

    //number of bits per byte
    //used for trasmitting the buffer through the serial port
    tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
    tty.c_cflag |= CS8; // 8 bits per byte

    //hardware flow control (CRTSCTS)
    // used to describe the method in which a serial device controls the amount of data being transmitted
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)

    //cread and clocal
    //CREAD - allows us to read data 
    //CLOCAL -  disables modem-specific signal, like DTR & RTS
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    //local modes (c_lflag)
    //two modes of input - canonical & non-canonical
    //canonical - input is processed when a new line character is received
    //The receiving application receives that data line-by-line. 
    tty.c_lflag &= ~ICANON; //  This is usually undesirable when dealing with a serial port, and so we normally want to disable canonical mode.

    //echo
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo

    //disable signal chars
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    //Input Modes (c_iflag)
    //Software Flow Control (IXOFF, IXON, IXANY)
    //IXON - Resume transmission 
    //IXOFF - Pause transmission 
    //IXANY - disables software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    //Disabling Special Handling Of Bytes On Receive
    //We just want the raw data
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    //Output Modes (c_oflag)
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed


    //VMIN and VTIME (c_cc)
    //VTIME - An internal timer is started when a character arrives, and it counts up in 0.1s
    //VMIN - is a character count ranging from 0 to 255 characters

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    //Baud rate
        cfsetispeed(&tty, baudRate);
    cfsetospeed(&tty, baudRate);

    //appyling to termios struct
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    //--------------------//
    //READING & WRITING
    //------------------//
    //writing
    // unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', ' ' , 'W', 'o', 'r', 'l', 'd', '\r' };
    // write(serial_port, msg, sizeof(msg));


    //reading
    //alloc memory for the buffer
    char readBuffer[256];
    int One, Two;
    //read incoming bytes
    int n = read(serial_port, &readBuffer, sizeof(readBuffer) -1); //n is number of read bytes

    while (1) {
        memset(readBuffer, 0, sizeof(readBuffer));
        ssize_t bytesRead = read(serial_port, readBuffer, sizeof(readBuffer) - 1);

        if (bytesRead > 0) {
            printf("Recieved: %s ", readBuffer);
            // Parse the read values into integers using sscanf
            sscanf(readBuffer, "%d %d", &One, &Two);
            printf("\r\n");
            printf("Values: %d %d\n", One, Two);
        } else if (bytesRead < 0) {
            printf("Error %i from open: %s\n", errno, strerror(errno));
            break;
        }
    }

    close(serial_port);
    return 0;
}