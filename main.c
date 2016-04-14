#include <sys/param.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/ioctl.h>


static struct termios tio;

static int fd;
/**
 *
 */
int
setupdev(int baudrate)
{
  int cflags = CS8 | CLOCAL | CREAD;

  switch(baudrate) {
  case 2400:
    cflags |= B2400;
    break;
  case 9600:
    cflags |= B9600;
    break;
  case 38400:
    cflags |= B38400;
    break;
  case 57600:
    cflags |= B57600;
    break;
  case 115200:
    cflags |= B115200;
    break;

  }


  tio.c_cflag = cflags;
  tio.c_iflag = IGNPAR;
  cfmakeraw(&tio);

  if(tcsetattr(fd, TCSANOW, &tio)) {
    perror("tcsetattr");
    return -1;
  }
  return 0;
}

static struct termios termio;


/**
 *
 */
static void
terminal(void)
{
  struct termios termio2;
  char buf[64];

  printf("Exit with ^B\n");

  if(!isatty(0)) {
    fprintf(stderr, "stdin is not a tty\n");
    exit(1);
  }
  if(tcgetattr(0, &termio) == -1) {
    perror("tcgetattr");
    exit(1);
  }
  termio2 = termio;
  termio2.c_lflag &= ~(ECHO | ICANON | ISIG);
  if(1) {
    if(tcsetattr(0, TCSANOW, &termio2) == -1)
      return;
  }

  struct pollfd fds[2];

  fds[0].fd = 0;
  fds[1].fd = fd;
  fds[0].events = POLLIN | POLLHUP;
  fds[1].events = POLLIN | POLLHUP;

  while(1) {
    poll(fds, 2, -1);

    if(fds[0].revents & (POLLERR | POLLHUP))
      break;
    if(fds[1].revents & (POLLERR | POLLHUP))
      break;

    if(fds[0].revents & POLLIN) {
      if(read(0, buf, 1) != 1) {
        perror("read");
        break;
      }
      if(buf[0] == 2)
        break;

      if(write(fd, buf, 1) != 1) {
        perror("write");
        break;
      }

    }

    if(fds[1].revents & POLLIN) {
      if(read(fd, buf, 1) != 1) {
        perror("read");
        break;
      }
      if(write(1, buf, 1) != 1) {
        perror("write");
        break;
      }
    }
  }


  tcsetattr(0, TCSANOW, &termio);
  printf("Exiting...\n");
  exit(0);
}


int
main(int argc, char **argv)
{
  const char *device = "/dev/ttyUSB0";
  int baudrate = 115200;

  if(argc >= 2) {
    device = argv[1];
  }
  if(argc >= 3) {
    baudrate = atoi(argv[2]);
  }

  fd = open(device, O_RDWR | O_NOCTTY);
  if(fd == -1) {
    perror("open serial port");
    exit(1);
  }

   setupdev(baudrate);

   terminal();
   return 0;
}
