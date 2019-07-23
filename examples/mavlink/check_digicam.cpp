#include <iostream>
#include <string>

#include "serial/serial.h"
#include "serial/impl/unix.h"

#include <common/mavlink.h>

mavlink_status_t status;
mavlink_message_t msg;
int chan = MAVLINK_COMM_1;

uint8_t buffer[MAVLINK_CORE_HEADER_LEN+1];

bool found = false;

int main()
{
  serial::Serial port("/dev/ttyACM0",115200,serial::Timeout::simpleTimeout(1000));
  if (port.isOpen())
  {
    std::cout << "Port opened successfully" << std::endl;
    while(!found)
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
              found = true;
              std::cout << "Heartbeat found" << std::endl;
            }
            break;
            default:
            break;
          }
        }
      }
    }

    found = false;

    mavlink_msg_command_long_pack(0,0,&msg,0,0,MAV_CMD_DO_DIGICAM_CONTROL,0,0,0,0,0,1,0,0);
    mavlink_msg_to_send_buffer(buffer, &msg);

    if (port.write(buffer,MAVLINK_CORE_HEADER_LEN+1) == MAVLINK_CORE_HEADER_LEN+1)
      std::cout << "Command sent" <<std::endl;
    else
      {
        std::cout << "Write error..." << std::endl;
        return -1;
      }

    while(!found)
    {
      if (port.available())
      {
        port.read(buffer, 1);

        if (mavlink_parse_char(chan, buffer[0], &msg, &status))
        {
          switch(msg.msgid)
          {
            case MAVLINK_MSG_ID_COMMAND_LONG:
            {
              mavlink_command_long_t command;
              mavlink_msg_command_long_decode(&msg, &command);
              if (command.command == MAV_CMD_DO_DIGICAM_CONTROL)
              {
                found = true;
                std::cout << "Digicam control feedback found" << std::endl;
              }
              else
              {
                std::cout << "Other command ID found: " << command.command << std::endl;
              }
            }
            break;

            case MAVLINK_MSG_ID_COMMAND_ACK:
            {
              mavlink_command_ack_t ack;
              mavlink_msg_command_ack_decode(&msg, &ack);
              if (ack.command == MAV_CMD_DO_DIGICAM_CONTROL)
              {
                found = true;
                std::cout << "Digicam control ack found" << std::endl;
              }
              else
              {
                std::cout << "Other ack ID found: " << ack.command << std::endl;
              }
            }
            break;

            case MAVLINK_MSG_ID_HEARTBEAT:
            {
              //std::cout << "Heartbeat still exists..." << std::endl;
            }
            default:
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
