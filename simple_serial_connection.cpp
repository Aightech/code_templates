#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <errno.h>   // Error integer and strerror() function
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()



int setSerialPort(const char* path)
{
  int serial_port = open(path, O_RDWR);

  struct termios tty;
  if(tcgetattr(serial_port, &tty) != 0)
    exit(1);

  tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;  // Clear stop field, only 1 stop bit (most common)
  tty.c_cflag &= ~CSIZE;   // Clear all bits that set the data size
  tty.c_cflag |= CS8;      // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow ctrl (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrlline(CLOCAL = 1)
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;   // Disable echo
  tty.c_lflag &= ~ECHOE;  // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
		   ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent spe. interp. of out bytes (newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conv of newline to car. ret/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10; // Wait for up to 1s, ret when any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  // Save tty settings, also checking for error
  if(tcsetattr(serial_port, TCSANOW, &tty) != 0)
    exit(0);
  return serial_port;
}


int
main()
{
  int serial_port = setSerialPort("/dev/ttyACM0");
  if(serial_port)
    exit(1);
    
  // Write to serial port
  unsigned char msg[] = "heyyyyyyy asd\ndd   ddddddddddddd";
  write(serial_port, msg, sizeof(msg));
    
  close(serial_port);
  return 0; // success
}
