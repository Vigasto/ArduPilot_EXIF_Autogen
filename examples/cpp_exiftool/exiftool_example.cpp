//------------------------------------------------------------------------------
// File:        example2.cpp
//
// Description: Simple example to write XMP:Title and XMP:Description to a file
//
// Syntax:      example2 FILE [TITLE] [DESCRIPTION]
//
// License:     Copyright 2013-2019, Phil Harvey (phil at owl.phy.queensu.ca)
//
//              This is software, in whole or part, is free for use in
//              non-commercial applications, provided that this copyright notice
//              is retained.  A licensing fee may be required for use in a
//              commercial application.
//
// Created:     2013-11-28 - Phil Harvey
//------------------------------------------------------------------------------

// Modified for generating GPS instead. ExifTool has been marked free to use as Perl is
// , stated in the website at the time of writing

#include <iostream>
#include <string>
#include <sstream>
#include "ExifTool.h"

#include <chrono>

template < typename Type > std::string to_str (const Type & t)
{
  std::ostringstream os;
  os << t;
  return os.str ();
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Example2: Write metadata to an image." << std::endl;
        std::cout << "Please specify file name" << std::endl;
        return 1;
    }
    const char *title = "default title";
    const char *desc = "default description";
    if (argc >= 3) title = argv[2];
    if (argc >= 4) desc = argv[3];

    // create our ExifTool object
    ExifTool *et = new ExifTool();

    auto started = std::chrono::high_resolution_clock::now();

    // 6 value below is a must; might add later
    // see http://owl.phy.queensu.ca/~phil/exiftool/TagNames/GPS.html
    et->SetNewValue("EXIF:GPSLatitude", "42.5");
    et->SetNewValue("EXIF:GPSLatitudeRef", "42.5");
    et->SetNewValue("EXIF:GPSLongitude", "42.5");
    et->SetNewValue("EXIF:GPSLongitudeRef", "42.5");
    et->SetNewValue("EXIF:GPSAltitude", "42.5");
    et->SetNewValue("EXIF:GPSLatitude", "42.5");


    // write the information
    et->WriteInfo(argv[1]);

    // wait for exiftool to finish writing
    int result = et->Complete(10); 

    auto done = std::chrono::high_resolution_clock::now();

    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count();

    if (result > 0) {
        // checking either the number of updated images or the number of update
        // errors should be sufficient since we are only updating one file,
        // but check both for completeness
        int n_updated = et->GetSummary(SUMMARY_IMAGE_FILES_UPDATED);
        int n_errors = et->GetSummary(SUMMARY_FILE_UPDATE_ERRORS);
        if (n_updated == 1 && n_errors <= 0) {
            std::cout << "Success!" << std::endl;
        } else {
            std::cerr << "The exiftool application was not successful." << std::endl;
        }
        // print any exiftool messages
        char *out = et->GetOutput();
        if (out) std::cout << out;
        char *err = et->GetError();
        if (err) std::cerr << err;
    } else {
        std::cerr << "Error executing exiftool command!" << std::endl;
    }
    delete et;
    return 0;
}

// end
