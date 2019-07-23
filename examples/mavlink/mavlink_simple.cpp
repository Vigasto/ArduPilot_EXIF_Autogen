#include <iostream>
#include <string>

#include "serial/serial.h"
#include "serial/impl/unix.h"

#include <common/mavlink.h>

mavlink_status_t status;
mavlink_message_t msg;
int chan = MAVLINK_COMM_1;

bool fix = false;

uint8_t buffer[0];

int main()
{
  serial::Serial port("/dev/ttyACM0",115200,serial::Timeout::simpleTimeout(1000));
  if (port.isOpen())
  {
    std::cout << "Port opened successfully" << std::endl;
    while(1)
    {
      if (port.available())
      {
        port.read(buffer, 1);

        if (mavlink_parse_char(chan, buffer[0], &msg, &status))
        {
          switch(msg.msgid)
          {
            case MAVLINK_MSG_ID_HEARTBEAT:
            {
              mavlink_heartbeat_t heartbeat;
              mavlink_msg_heartbeat_decode(&msg, &heartbeat);
              std::cout << "Heartbeat found" << std::endl;
            }
            break;

            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            {
             if (fix)
             {
               mavlink_global_position_int_t pos;
               mavlink_msg_global_position_int_decode(&msg, &pos);
               std::cout << "GPS found" << std::endl;
               std::cout << "Lat: " << pos.lat << std::endl;
               std::cout << "Lon: " << pos.lon << std::endl;
               std::cout << "Alt: " << pos.lat << std::endl;
               std::cout << "RelAlt: " << pos.relative_alt / 1000 << std::endl;
             }
            }
	    break;

	    case MAVLINK_MSG_ID_GPS_RAW_INT:
	    {
	     mavlink_gps_raw_int_t status;
	     mavlink_msg_gps_raw_int_decode(&msg, &status);
	     if (status.fix_type >=2)
	       fix = true;
	     else
	       fix = false;
	     std::cout << "Fix Type: " << (int)status.fix_type << std::endl;
	     std::cout << "Visible Satellites: " << (int)status.satellites_visible << std::endl;
            }
            break;
	  }
        }
      }
    }
  }
  else
    std::cout << "Port isn't open..." << std::endl;
  return 0;
}
