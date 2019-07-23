ArduPilot-based EXIF AutoGen
========================
 Complementary script for capturing photos with automatically generated geotags in EXIF format with ArduPilot or MAVLink based systems.


## Dependencies

- g++
- cmake
- git
-  serial library of your OS
-  gphoto2
-  exiftool

## Build Instructions

### Setup project root

- `sudo apt-get install g++ build-essential git cmake gphoto2` (replace gphoto2 to any packages supported in your distro)

- `git clone https://github.com/Vigasto/AP_EXIF_Autogen`

- `cd AP_EXIF_Autogen/`

- `git submodule update --init --recursive`

### Build gphoto2pp 

In `lib/gphoto2pp` directory :

- `./cmake_release.sh`

- `cd build/release/`

- `make -j<number of threads>`

- `sudo make install`

### Build exiftool

- Refer to http://owl.phy.queensu.ca/~phil/exiftool/install.htm

### Build project

In project root directory :

- `mkdir build && cd build`
- `cmake ..`
- `make -j <threads>`

## Notes

- If working with distros with gphoto <=2.5, omit version checking in its CMakeLists.txt; comment this snippet : 

  ```
	if(${GPHOTO2_VERSION_STRING} VERSION_LESS "2.5")
		add_definitions("-DGPHOTO_LESS_25")
	endif()
```

- Gphoto2pp does not correctly move its headers to /usr/local/include while installing. Do : 
 
 `sudo mv <gphoto2pp directory>/include/gphoto2pp/ /usr/local/include/`
 
 after `make install`
