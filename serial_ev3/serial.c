#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

int main(int argc, char **argv) {
  
  if (argc < 2) {
    printf("Please supply a port!\n");
    return 1;
  }

  printf("Opening port %s\n", argv[1]);
  int pico_fd = open(argv[1] , O_RDWR | O_NOCTTY | O_SYNC);

  if (pico_fd < 0) {
    printf("Failed to open %s\n", argv[1]);
    return 1;
  }

  struct termios tty;

  if (tcgetattr(pico_fd, &tty) != 0) {
    perror("Error: failed to get attributes");
    close(pico_fd);
    return 1;

  }

  tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(pico_fd, TCSANOW, &tty) != 0) {
    perror("Error: failed to set terminal attributes");
    close(pico_fd);
    return 1;
  }


  tcflush(pico_fd, TCIFLUSH);
  sleep(1);
  char buf[32];
  while (1) {
    char cmd = 'd';
    write(pico_fd, &cmd, 1);

    int n = read(pico_fd, buf, 32);

    //float *f = buf;
    //printf("%f\n", *f);
    printf("RECEIVED (%d bytes): %s\n", n , buf);
    usleep(100000);
  }

  close(pico_fd);
  return 0;
}
