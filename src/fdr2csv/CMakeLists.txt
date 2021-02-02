cmake_minimum_required(VERSION 3.5)
project(fdr2csv LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
        AFTER
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/src/commandline"
        "${CMAKE_SOURCE_DIR}/../fbw/src/model"
        "${CMAKE_SOURCE_DIR}/../fbw/src/zlib"
)

add_executable(
        fdr2csv
        ../fbw/src/zlib/adler32.c
        ../fbw/src/zlib/crc32.c
        ../fbw/src/zlib/deflate.c
        ../fbw/src/zlib/gzclose.c
        ../fbw/src/zlib/gzlib.c
        ../fbw/src/zlib/gzread.c
        ../fbw/src/zlib/gzwrite.c
        ../fbw/src/zlib/infback.c
        ../fbw/src/zlib/inffast.c
        ../fbw/src/zlib/inflate.c
        ../fbw/src/zlib/inftrees.c
        ../fbw/src/zlib/trees.c
        ../fbw/src/zlib/zfstream.cc
        ../fbw/src/zlib/zutil.c
        src/commandline/CommandLine.cpp
        src/FlightDataRecorderConverter.cpp
        src/main.cpp
)
