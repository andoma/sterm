#include <netinet/in.h>
#include <arpa/inet.h>
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
#include <sys/ioctl.h>

#include <termios.h>

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
  case 19200:
    cflags |= B19200;
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
  default:
    printf("Baudrate %d not supported\n", baudrate);
    return -1;
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

static int g_hex_mode;
static int g_line_mode;
static int g_lf_sends_cr;

/**
 *
 */
static void
terminal(void)
{
  uint8_t buf[64];

  printf("Exit with ^%c\n", g_line_mode ? 'C' : 'B');

  if(!isatty(0)) {
    fprintf(stderr, "stdin is not a tty\n");
    exit(1);
  }
  if(tcgetattr(0, &termio) == -1) {
    perror("tcgetattr");
    exit(1);
  }
  if(!g_line_mode) {

    struct termios termio2;
    termio2 = termio;
    termio2.c_lflag &= ~(ECHO | ICANON | ISIG);

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
      if(!g_line_mode && buf[0] == 2)
        break;

      if(buf[0] == 10 && g_lf_sends_cr)
        buf[0] = 13;

      if(write(fd, buf, 1) != 1) {
        perror("write");
        break;
      }
    }

    if(fds[1].revents & POLLIN) {
      int r = read(fd, buf, sizeof(buf));
      if(r == 0) {
        break;
      }
      if(r < 0) {
        perror("read");
        break;
      }
      if(g_hex_mode) {
        for(int i = 0; i < r; i++) {
          char hex[8];
          snprintf(hex, sizeof(hex), "%02x '%c'\n", buf[i],
                   buf[i] >= 32 && buf[i] < 128 ? buf[i] : '.');
          if(write(1, hex, 7) != 7) {
            perror("write");
            break;
          }
        }

      } else {
        if(write(1, buf, r) != r) {
          perror("write");
          break;
        }
      }
    }
  }


  tcsetattr(0, TCSANOW, &termio);
  printf("Exiting...\n");
  exit(0);
}



static void
usage(const char *argv0)
{
  printf("Usage: %s OPTIONS\n\n", argv0);
  printf("   -d DEVICE    [/dev/ttyUSB0]\n");
  printf("   -b BAUDRATE  [115200]\n");
  printf("   -c HOST:PORT Connect to TCP HOST:PORT\n");
  printf("\n");
  printf("   -R Toogle RTS on start\n");
  printf("   -D Toogle DTR on start\n");
  printf("   -H Output hex values\n");
  printf("   -l Line mode (including local echo)\n");
  printf("   -C LF sends CR\n");
  printf("\n");
}


#ifdef __linux__
struct sockaddr_l2 {
  sa_family_t l2_family;
  uint16_t l2_psm;
  uint8_t l2_bdaddr[6];
  uint16_t l2_vid;
  uint8_t l2_bdaddr_type;
};

#define BDADDR_BREDR		0x00
#define BDADDR_LE_PUBLIC	0x01
#define BDADDR_LE_RANDOM	0x02


static int
hexnibble(const char **s)
{
  while(1) {
    char c = **s;
    if(c == 0)
      return -1;

    *s = *s + 1;
    switch(c) {
    case '0' ... '9':
      return c - '0';
    case 'a' ... 'f':
      return c - 'a' + 10;
    case 'A' ... 'F':
      return c - 'A' + 10;
    }
  }
}


#endif

int
main(int argc, char **argv)
{
  const char *device = "/dev/ttyUSB0";
  int baudrate = 115200;
  char *hostport = NULL;
  int toggle_dtr = 0;
  int toggle_rts = 0;
  int opt;
  const char *bleaddr = NULL;

  while((opt = getopt(argc, argv, "d:b:RDhHc:lCB:")) != -1) {
    switch(opt) {
    case 'b':
      baudrate = atoi(optarg);
      break;
    case 'B':
      bleaddr = optarg;
      break;
    case 'd':
      device = optarg;
      break;
    case 'R':
      toggle_rts = 1;
      break;
    case 'D':
      toggle_dtr = 1;
      break;
    case 'H':
      g_hex_mode = 1;
      break;
    case 'l':
      g_line_mode = 1;
      break;
    case 'C':
      g_lf_sends_cr = 1;
      break;
    case 'c':
      hostport = optarg;
      break;
    case 'h':
      usage(argv[0]);
      exit(0);
    default:
      usage(argv[0]);
      exit(1);
    }
  }

  if(bleaddr != NULL) {
#ifdef __linux__

    fd = socket(PF_BLUETOOTH, SOCK_STREAM, 0);
    if(fd == -1) {
      perror("socket");
      exit(1);
    }

    struct sockaddr_l2 addr = {0};

    addr.l2_family = AF_BLUETOOTH;
    addr.l2_bdaddr_type = BDADDR_LE_RANDOM;
    if(bind(fd, (struct sockaddr *) &addr, sizeof(addr)) == -1) {
      perror("bind");
      exit(1);
    }

    for(int i = 5 ; i >= 0; i--) {
      int h = hexnibble(&bleaddr);
      int l = hexnibble(&bleaddr);
      if(h == -1 || l == -1) {
        fprintf(stderr, "Malformed address\n");
        exit(1);
      }
      addr.l2_bdaddr[i] = (h << 4) | l;
    }


    addr.l2_bdaddr_type = BDADDR_LE_RANDOM;
    addr.l2_psm = 0x80 + 23;
    if(*bleaddr == ':')
      addr.l2_psm = atoi(bleaddr + 1);

    fprintf(stderr, "Connecting to %02x:%02x:%02x:%02x:%02x:%02x:%d\n",
            addr.l2_bdaddr[5],
            addr.l2_bdaddr[4],
            addr.l2_bdaddr[3],
            addr.l2_bdaddr[2],
            addr.l2_bdaddr[1],
            addr.l2_bdaddr[0],
            addr.l2_psm);

    if(connect(fd, (struct sockaddr *) &addr, sizeof(addr)) == -1) {
      perror("connect");
      exit(1);
    }

    terminal();
    return 0;

#else
    fprintf(stderr, "Bluetooth not supported on this platform\n");
    exit(1);
#endif
  }


  if(hostport != NULL) {

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd == -1) {
      perror("socket");
      exit(1);
    }

    int port = 3000;

    char *portstr = strchr(hostport, ':');
    if(portstr != NULL) {
      port = atoi(portstr + 1);
      *portstr = 0;
    }

    struct sockaddr_in sin = {
      .sin_family = AF_INET,
      .sin_port = htons(port),
      .sin_addr.s_addr = inet_addr(hostport)
    };

    if(connect(fd, (struct sockaddr *)&sin, sizeof(sin)) == -1) {
      perror("connect");
      exit(1);
    }
    terminal();
    return 0;
  }


  fd = open(device, O_RDWR | O_NOCTTY);
  if(fd == -1) {
    perror("open serial port");
    exit(1);
  }

   setupdev(baudrate);

   if(toggle_dtr) {
     // Turn on DTR
     printf("Toggle DTR\n");
     int f = TIOCM_DTR;
     ioctl(fd, TIOCMBIC, &f);
     usleep(1000);
     ioctl(fd, TIOCMBIS, &f);
   }

   if(toggle_rts) {
     // Turn on RTS
     printf("Toggle RTS\n");
     int f = TIOCM_RTS;
     ioctl(fd, TIOCMBIS, &f);
     usleep(1000);
     ioctl(fd, TIOCMBIC, &f);
   }


   terminal();
   return 0;
}
