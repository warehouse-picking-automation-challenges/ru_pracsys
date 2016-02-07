// Uses POSIX functions to send and receive data from a Maestro.
// NOTE: The Maestro's serial mode must be set to "USB Dual Port".
// NOTE: You must change the 'const char * device' line below.
 
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
 
#include <termios.h>
 
// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition(int fd, unsigned char channel)
{
  unsigned char command[2];
  
  command[0] = 0x90;
  command[1] = channel;
  if(write(fd, command, 2) == -1)
  {
    printf("Error wrinting in maestroGetPosition()\n");
    perror("error writing");
    return -1;
  }
  
  getchar();
  unsigned char response[2];
  int bytes_read;
  if((bytes_read = read(fd, response, 2)) != 2)
  {
    printf("Error reading in maestroGetPosition(). %d bytes read.\n", bytes_read);
    //perror("error reading");
    return -1;
  }
   
  return response[0] + 256*response[1];
}
 
// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}
 
int main()
{
  // Open the Maestro's virtual COM port.
  const char * device = "/dev/ttyACM0";  // Linux
  int fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    printf("Error opening\n");
    perror(device);
    return 1;
  }
    
  printf("Try reading pos...\n");
  int position = maestroGetPosition(fd, 5);
  printf("Current position is %d.\n", position);
 
  int target = (position < 6000) ? 7000 : 5000;
  printf("Setting target to %d (%d us).\n", target, target/4);
  maestroSetTarget(fd, 5, target);
   
  close(fd);
  return 0;
}
