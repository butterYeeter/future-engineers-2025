#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <stdlib.h>

int pico_fd;

int init_usb(char *port, int baudrate) {
  printf("Opening port %s\n", port);
  pico_fd = open(port , O_RDWR | O_NOCTTY | O_SYNC);

  if (pico_fd < 0) {
    printf("Failed to open %s\n", port);
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
  tty.c_cc[VMIN] = 4;
  tty.c_cc[VTIME] = 1;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(pico_fd, TCSANOW, &tty) != 0) {
    perror("Error: failed to set terminal attributes");
    close(pico_fd);
    return 1;
  }

  return 0;
}

void flush_input() {
  tcflush(pico_fd, TCIFLUSH);
  sleep(1);
}

int write_usb(char buf[], int len) {
  return write(pico_fd, buf, len);
}

int read_usb(char buf[], int len) {
  return read(pico_fd, buf, len);
}

void deinit() {
  close(pico_fd);
}

float angle() {
  char buf[32];
  char cmd = 'd';
  int n = write(pico_fd, &cmd, 1);
  printf("wrote %d bytes\n", n);

  n = read(pico_fd, buf, sizeof(buf));
  printf("Read %d bytes\n", n);
  return atof(buf);
}
