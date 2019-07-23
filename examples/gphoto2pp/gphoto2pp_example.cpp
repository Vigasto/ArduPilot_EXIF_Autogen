#include <gphoto2pp/camera_wrapper.hpp> // Header for CameraWrapper include
#include <gphoto2pp/camera_file_wrapper.hpp> // Header for CameraFileWrapper include
#include <gphoto2pp/camera_file_path_wrapper.hpp>
#include <gphoto2pp/camera_capture_type_wrapper.hpp>
#include <gphoto2pp/camera_file_type_wrapper.hpp>

#include <chrono>

#include <iostream>

int main()
{
	// Connects to the first camera found and initializes
	gphoto2pp::CameraWrapper camera;

	// Prints out the summary of your camera's abilities
	std::cout << camera.getSummary() << std::endl;

	auto start = std::chrono::high_resolution_clock::now();
	// Takes a picture with the camera and wait event for the capture to finish

	auto cameraFilePath = camera.capture(gphoto2pp::CameraCaptureTypeWrapper::Image);
        auto capture = std::chrono::high_resolution_clock::now();

	auto cameraFile = camera.fileGet(cameraFilePath.Folder, cameraFilePath.Name, gphoto2pp::CameraFileTypeWrapper::Normal); 

	cameraFile.save(cameraFilePath.Name);
	auto finish = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(capture - start).count() << std::endl;
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(finish - capture).count() << std::endl;
	return 1;
}
