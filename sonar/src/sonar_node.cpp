#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "ros/ros.h"
#include "sonar/Sonar_raw.h"

#define SERIAL_PORT "/dev/ttyAMA0"
#define MAXSIZE 60

sonar::Sonar_raw sonar_raw;
/*ringbuffer*/  
short ringbuf[MAXSIZE];
short readbuf[18];
unsigned short  data[9];
int read_addr = 0;  
int write_addr = 0;  
unsigned short crc_data= 0;

int next_data_handle(int addr)     
{     
  return (addr + 1) == MAXSIZE ?  0 : (addr + 1) ;     
}
     
int next_data_handle(int addr , int count)     
{     
  int a;
  a = addr;
  for(int i = 0; i < count ; i++)
  { 
    a = ( (a + 1)  == MAXSIZE ?  0 : ( a + 1 ) ) ;   
  }
  return a;  
}
 
void write_data(short data)  
{  
  *(ringbuf+write_addr) = data;  
  write_addr = next_data_handle(write_addr);  
}  

/*crc*/
unsigned short crc_update(unsigned short  crc ,  unsigned char data)
{
  data ^= (crc & 0xff) ;
  data ^= data << 4;
  return ((((unsigned short)data << 8) | ((crc >> 8) & 0xff) )^ (unsigned char)(data >> 4) ^ ((unsigned short)data << 3)) ;
}

unsigned short crc(void* data, unsigned short count)
{
  unsigned short crc = 0xff;
  unsigned char *ptr = (unsigned char*)data;
  for(int i = 0 ; i < count ; i++)
  {
    crc = crc_update(crc , *ptr);
    ptr++;
  }
  return crc;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sonar_node");
  ros::NodeHandle n;
  ros::Publisher sonar_pub = n.advertise<sonar::Sonar_raw>("sonar_data", 10);
  ros::Rate loop_rate(50);

  int fd ;
  short ret ;
  if ((fd = serialOpen ( SERIAL_PORT, 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  while(ros::ok())
  {
    for(int i = 0 ; i < 29 ; i++)
    {
      ret = serialGetchar(fd) ;
      if( ret < 0 )
      {
        fprintf (stderr, "No data : %s\n", strerror (errno)) ;
        return 1;
      }else
      {
        write_data(ret) ;
//        ROS_INFO("test_1:%X",ret); 
      }
    }
    for(int i = 0 ; i < MAXSIZE ; i++)
    {
      if((ringbuf[read_addr] == '>') && (ringbuf[next_data_handle(read_addr)] == '*') && (ringbuf[next_data_handle(read_addr,2)] == '>') 
      && (ringbuf[next_data_handle(read_addr,3)] == 0x12) && (ringbuf[next_data_handle(read_addr,4)] == 0x00) && (ringbuf[next_data_handle(read_addr,5)] == 'c')
      && (ringbuf[next_data_handle(read_addr,26)] == '<') && (ringbuf[next_data_handle(read_addr,27)] == '#') && (ringbuf[next_data_handle(read_addr,28)] == '<'))  
      {
        read_addr = next_data_handle(read_addr,6) ; 
        for(int j = 0 ; j < 18 ; j++)
        {
          readbuf[j] = ringbuf[read_addr] ;
          read_addr = next_data_handle(read_addr) ;
//          ROS_INFO("test_2:%X",readbuf[j]) ; 
        }
        break;
      }else
      {
        read_addr = next_data_handle(read_addr) ;
      }
    }
    crc_data =  ringbuf[read_addr] | (ringbuf[next_data_handle(read_addr)] << 8) ;
    ROS_INFO("test_3:%X",crc_data); 
    data[0] = (readbuf[1]<<8) | readbuf[0] ;
    data[1] = (readbuf[3]<<8) | readbuf[2] ; 
    data[2] = (readbuf[5]<<8) | readbuf[4] ; 
    data[3] = (readbuf[7]<<8) | readbuf[6] ; 
    data[4] = (readbuf[9]<<8) | readbuf[8] ; 
    data[5] = (readbuf[11]<<8) | readbuf[10] ; 
    data[6] = (readbuf[13]<<8) | readbuf[12] ; 
    data[7] = (readbuf[15]<<8) | readbuf[14] ; 
    data[8] = (readbuf[17]<<8) | readbuf[16] ;

    read_addr = next_data_handle(read_addr, 5) ;
//    ROS_INFO("%d" , data[0]) ;
 //   ROS_INFO("%d" , data[1]) ;
//    ROS_INFO("%d" , data[2]) ;
//    ROS_INFO("%d" , data[3]) ;
//    ROS_INFO("%d" , data[4]) ;
//    ROS_INFO("%d" , data[5]) ;
//    ROS_INFO("%d" , data[6]) ;
//    ROS_INFO("%d" , data[7]) ;
//    ROS_INFO("%d" , data[8]) ;
    sonar_raw.sonar_1 = 0;
    sonar_raw.sonar_2 = 0;
    sonar_raw.sonar_3 = 0;
    sonar_raw.sonar_4 = 0;
    sonar_raw.sonar_5 = 0;
    sonar_raw.sonar_6 = 0;
    sonar_raw.sonar_7 = 0;
    sonar_raw.sonar_8 = 0;
    sonar_raw.sonar_9 = 0;
   if(crc(data,18) == crc_data)
    {
      sonar_raw.sonar_1 = data[0];
      sonar_raw.sonar_2 = data[1];
      sonar_raw.sonar_3 = data[2];
      sonar_raw.sonar_4 = data[3];
      sonar_raw.sonar_5 = data[4];
      sonar_raw.sonar_6 = data[5];
      sonar_raw.sonar_7 = data[6];
      sonar_raw.sonar_8 = data[7];
      sonar_raw.sonar_9 = data[8];
    }
    sonar_pub.publish(sonar_raw);
    ros:: spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

