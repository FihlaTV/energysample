/*
Wifi channel energy sampler driver and test program.
Paul Gardner-Stephen
Copyright (C) 2015 Flinders University

This program sends a series of pulses to power a wifi energy sampler
using a serial port.  Byte $00 yields all low.

It then listens for chirps from the energy sampler to indicate that 
something seems to be happening on the channel.

It also listens to the wifi interface at the same time using libpcap
to see when packets are visible.

It then does some basic statistics on the result to see how well
correlated they are, i.e., whether the two events happen together, or
whether there are false positives for the energy sampler, or false
negatives where the energy sampler fails to notice an actual packet.

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
#include <pcap.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/if_ether.h>
#include <poll.h>
#include <signal.h>

int fd=-1;
int count_last_second=0;
int count_this_second=0;
long last_seconds=0;

void alarm_handler(int signal_number)
{
  unsigned char zero = 0x00;
  if (fd!=-1) write(fd,&zero,1);

  struct timeval tv;
  gettimeofday(&tv,NULL);
  if (tv.tv_sec!=last_seconds) {
    count_last_second=count_this_second;
    count_this_second=1;
    last_seconds=tv.tv_sec;
  } else count_this_second++;
  
  signal(SIGALRM, alarm_handler);
}

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
  case 57600: baud_rate = B57600; break;
  case 115200: baud_rate = B115200; break;
  case 230400: baud_rate = B230400; break;
  default:
    return -1;
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
  if (argc<=4) {
    fprintf(stderr,"usage: sudo %s serialport networkinterface portspeed dutycycle\n",argv[0]);
    fprintf(stderr,"       port_speed affects is the pulse frequency used to power the sampler.\n");
    fprintf(stderr,"       Valid values are resticted by available serial port speeds.\n");
    fprintf(stderr,"       USB serial port adapters may clump characters resulting in strange problems.\n");
    fprintf(stderr,"       Pulse width will be 10x port speed.  Duty cycle will be calculated from that.\n");
    exit(-1);
  }

  fd=open(argv[1],O_RDONLY);
  if (fd<0) {
    fprintf(stderr,"Failed to open serial port '%s'\n",argv[1]);
    exit(-1);
  }
  
  int port_speed=atoi(argv[3]);
  if (setup_port(fd,port_speed)==-1) {
    fprintf(stderr,"Illegal serial port speed.\n");
    exit(-1);
  }
  
  int duty_cycle=atoi(argv[4]);
  // 1% duty cycle is sending one character every 1000 serial ticks.
  // This is because one character consists of 10 bits (start+data+stop).
  if (duty_cycle<1||duty_cycle>75) {
    fprintf(stderr,"Duty cycle should be between 1 and 75 percent.\n");
    fprintf(stderr,"(values >75% may not work on some serial ports, due to lack of inter-character spacing)\n");
    exit(-1);
  }
  int pulse_rate = port_speed/10.0/100.0*duty_cycle;
  printf("Aiming to send %d pulses per second to match %d%% duty cycle.\n",
	 pulse_rate,duty_cycle);
  int pulse_interval=1000000/pulse_rate;
  printf("Energy sampler will be powered for %.3fms every %.3fms\n",
	 1000.0 * 10.0 / port_speed, pulse_interval/1000.0);
  
  char buffer[1024];

  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_t* descr;
  const u_char *packet;
  struct pcap_pkthdr packet_header;     /* pcap.h */
  
  descr = pcap_open_live(argv[2],BUFSIZ,0,1,errbuf);
  
  if(descr == NULL)
    {
      printf("pcap_open_live(): %s\n",errbuf);
      exit(1);
    }
  
  // int last_time=time_in_usec();

  int pcap_fd=pcap_get_selectable_fd(descr);
  pcap_setnonblock(descr,1,errbuf);
  
  struct pollfd fds[2];

  fds[0].fd = pcap_fd;
  fds[0].events = POLLIN;
  fds[1].fd = fd;
  fds[1].events = POLLIN;

  printf("Ready (fds=%d,%d).\n",pcap_fd,fd);
  alarm_handler(-1);

  struct itimerval itv;
  // Call every 1ms
  itv.it_interval.tv_usec = pulse_interval;
  itv.it_interval.tv_sec = 0;
  itv.it_value.tv_usec = pulse_interval;
  itv.it_value.tv_sec = 0;
  setitimer(ITIMER_REAL,&itv,NULL);

  
  while(1) {
    poll(fds,2,10);
    if (fds[0].revents)
      {
	packet = pcap_next(descr,&packet_header);
	if (packet) { 
	  printf("wifi: %d (%d ticks last second)\n",
		 time_in_usec(),count_last_second);
	  fflush(stdout);
	}
      }
    if (fds[1].revents) {
      int r=read(fd,buffer,1024);
      if (r>0) {
	printf("sampler: %d (%d events) (%d ticks last second)\n",
	       time_in_usec(),r, count_last_second);
	fflush(stdout);
      }
    }
  }
}
