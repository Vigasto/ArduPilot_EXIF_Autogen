#include <iostream>
#include <string>
#include <chrono>

#include "serial/serial.h"
#include "serial/impl/unix.h"

#include <common/mavlink.h>

mavlink_status_t status;
mavlink_message_t msg;
int chan = MAVLINK_COMM_1;

bool fix = false;

uint8_t buffer[0];

std::string mavlink_lookup_single()
{
  bool found = false;
  std::vector<serial::PortInfo> devices_found = serial::list_ports();

  std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while( iter != devices_found.end() && !found )
  {
    serial::PortInfo device = *iter++;
    std::cout << device.port << device.description << device.hardware_id << std::endl;
    serial::Serial port(device.port, 115200, serial::Timeout::simpleTimeout(1000));
    if (port.isOpen())
    {
      std::cout << "Port candidate opened" << std::endl;
      auto start = std::chrono::high_resolution_clock::now();
      auto stop = std::chrono::high_resolution_clock::now();
      while ((std::chrono::duration_cast<std::chrono::milliseconds>(stop - start)).count() <= 10000)
      {
        if (port.available())
        {
          port.read(buffer, 1);

          if (mavlink_parse_char(chan, buffer[0], &msg, &status))
          {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
              {
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                std::cout << "MAVLink source found" << std::endl;
		return device.port;
              }
          }
        }
        stop = std::chrono::high_resolution_clock::now();
      }
    }
  }
  std::cerr << "No sources found!" << std::endl;
  return "";
}

int main()
{
  std::string port_name = mavlink_lookup_single();
  if (port_name == "")
    return 1;
  serial::Serial port(port_name,115200,serial::Timeout::simpleTimeout(1000));
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
