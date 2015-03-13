/*
Serval DNA packet radio interface
Copyright (C) 2013 Serval Project Inc.
Copyright (C) 2013 Paul Gardner-Stephen
 
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
 
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
 
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>

int time_in_usec()
{
  struct timeval tv;

  gettimeofday(&tv,NULL);
  return tv.tv_usec;
}

int setup_port(int fd,int speed)
{
  struct termios t;

  if (tcgetattr(fd, &t))
  { perror("tcgetattr failed"); exit(-1); }
  speed_t baud_rate;
  switch(speed){
  case 0: baud_rate = B0; break;
  case 50: baud_rate = B50; break;
  case 75: baud_rate = B75; break;
  case 110: baud_rate = B110; break;
  case 134: baud_rate = B134; break;
  case 150: baud_rate = B150; break;
  case 200: baud_rate = B200; break;
  case 300: baud_rate = B300; break;
  case 600: baud_rate = B600; break;
  case 1200: baud_rate = B1200; break;
  case 1800: baud_rate = B1800; break;
  case 2400: baud_rate = B2400; break;
  case 4800: baud_rate = B4800; break;
  case 9600: baud_rate = B9600; break;
  case 19200: baud_rate = B19200; break;
  case 38400: baud_rate = B38400; break;
  default:
  case 57600: baud_rate = B57600; break;
  case 115200: baud_rate = B115200; break;
  case 230400: baud_rate = B230400; break;
  }

  if (cfsetospeed(&t, baud_rate))
    perror("Failed to set output baud rate");
  if (cfsetispeed(&t, baud_rate))
    perror("Failed to set input baud rate");

  // 8N1
  t.c_cflag &= ~PARENB;
  t.c_cflag &= ~CSTOPB;
  t.c_cflag &= ~CSIZE;
  t.c_cflag |= CS8;

  t.c_lflag &= ~(ICANON | ISIG | IEXTEN | ECHO | ECHOE);
  /* Noncanonical mode, disable signals, extended
   input processing, and software flow control and echoing */
  
  t.c_iflag &= ~(BRKINT | ICRNL | IGNBRK | IGNCR | INLCR |
		 INPCK | ISTRIP | IXON | IXOFF | IXANY | PARMRK);
  /* Disable special handling of CR, NL, and BREAK.
   No 8th-bit stripping or parity error handling.
   Disable START/STOP output flow control. */
  
  // Enable/disable CTS/RTS flow control
  // t.c_cflag &= ~CNEW_RTSCTS;

  // no output processing
  t.c_oflag &= ~OPOST;

  if (tcsetattr(fd, TCSANOW, &t))
    perror("Failed to set terminal parameters");
  
  return 0;
}


int main(int argc,char **argv)
{
  int fd=open(argv[1],O_RDONLY);
  setup_port(fd,115200);
  char buffer[1024];

  int last_time=time_in_usec();
  while(1) {
    int r=read(fd,buffer,1);
    int interval=time_in_usec()-last_time;
    if (interval<0) interval+=1000000;
    printf("%d\n",interval);
    last_time=time_in_usec();
  }
}
