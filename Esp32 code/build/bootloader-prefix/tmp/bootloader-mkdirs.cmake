# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/duvan/esp-idf-v5.1.1/components/bootloader/subproject"
  "D:/He thong nhung/New Folder/readSensor_adaf/build/bootloader"
  "D:/He thong nhung/New Folder/readSensor_adaf/build/bootloader-prefix"
  "D:/He thong nhung/New Folder/readSensor_adaf/build/bootloader-prefix/tmp"
  "D:/He thong nhung/New Folder/readSensor_adaf/build/bootloader-prefix/src/bootloader-stamp"
  "D:/He thong nhung/New Folder/readSensor_adaf/build/bootloader-prefix/src"
  "D:/He thong nhung/New Folder/readSensor_adaf/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/He thong nhung/New Folder/readSensor_adaf/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/He thong nhung/New Folder/readSensor_adaf/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
