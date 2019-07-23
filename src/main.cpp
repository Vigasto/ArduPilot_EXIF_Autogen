// Modified for generating GPS instead. ExifTool has been marked free to use as
// Perl is, stated in the website at the time of writing

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <thread>

#include "ExifTool.h"

#include <gphoto2pp/camera_wrapper.hpp>                 // Header for CameraWrapper
#include <gphoto2pp/camera_file_wrapper.hpp>    // Header for CameraFileWrapper
#include <gphoto2pp/camera_file_path_wrapper.hpp>
#include <gphoto2pp/camera_capture_type_wrapper.hpp>
#include <gphoto2pp/camera_file_type_wrapper.hpp>

#include "serial/serial.h"
#include "serial/impl/unix.h"

#include <common/mavlink.h>

mavlink_status_t status;
mavlink_message_t msg;
int chan = MAVLINK_COMM_1;
bool fix = false;
uint8_t buffer[0];

// Replacement to std::to_string() because accuracy issues
// see : https://stackoverflow.com/questions/2125880/convert-float-to-stdstring-in-c
template < typename Type > std::string to_str (const Type & t)
{
  std::ostringstream os;
  os << t;
  return os.str ();
}

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

int write_exif(std::string fileName, mavlink_global_position_int_t pos)
{
  ExifTool *et = new ExifTool();

  //source https://sno.phy.queensu.ca/~phil/exiftool/TagNames/GPS.html
  et->SetNewValue("EXIF:GPSLatitude", to_str(pos.lat/10000000.0).c_str());
  et->SetNewValue("EXIF:GPSLatitudeRef", to_str(pos.lat/10000000.0).c_str());
  et->SetNewValue("EXIF:GPSLongitude", to_str(pos.lon/10000000.0).c_str());
  et->SetNewValue("EXIF:GPSLongitudeRef", to_str(pos.lon/10000000.0).c_str());
  et->SetNewValue("EXIF:GPSAltitude", to_str(pos.alt/1000.0).c_str());
  et->SetNewValue("EXIF:GPSLatitude", to_str(pos.alt/1000.0).c_str());

  et->WriteInfo(fileName.c_str());
  int result = et->Complete(10);

  if (result > 0)
  {
    delete et;
    return 0;
  }
  delete et;
  std::cerr << "Error executing exiftool command!" << std::endl;
  return 2;
}

int main()
{
  int count = 0;
  std::vector<std::thread> exifJobs;
  mavlink_global_position_int_t pos;

  std::string port_name = mavlink_lookup_single();
  if (port_name == "")
    return 1;
  serial::Serial port(port_name,115200,serial::Timeout::simpleTimeout(1000));
  // Connects to the first camera found and initializes

  gphoto2pp::CameraWrapper camera;

  while(port.isOpen())
  {
    if(port.available())
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
            // ArduPilot uses MV_CMD_DO_DIGICAM_CONTROL for both capture command and feedback
            // see : https://github,com/ArduPilot/ardupilot/blob/master/libraries/AP_Camera/AP_Camera.cpp
            if (command.command == MAV_CMD_DO_DIGICAM_CONTROL)
            {
              auto cameraFilePath = camera.capture(gphoto2pp::CameraCaptureTypeWrapper::Image);
              auto cameraFile = camera.fileGet(cameraFilePath.Folder, cameraFilePath.Name, gphoto2pp::CameraFileTypeWrapper::Normal); 

	      // Add zero padding until 7 digits
 	      std::stringstream fileName;
              fileName << std::setfill('0') << std::setw(7) << ++count;
              std::string paddedName = fileName.str();

              cameraFile.save(paddedName);

              exifJobs.push_back(std::thread(write_exif, paddedName, pos));
            }
          }
          break;

          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          {
            if (fix)
              mavlink_msg_global_position_int_decode(&msg, &pos);
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
          }
          break;
        }
      }
    }
    for (auto &i : exifJobs)
      if (i.joinable())
        i.join();
  }
  for (auto &i : exifJobs)
    if (i.joinable())
      i.join();
  return 0;
}
