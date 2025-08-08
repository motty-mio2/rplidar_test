#ifndef FLAGS_HPP
#define FLAGS_HPP

#include <gflags/gflags.h>

DEFINE_string(d, "/dev/ttyUSB0", "path/to/device");
DEFINE_uint32(num, 4, "number of areas");
DEFINE_double(max_dist, 1000, "maximum distance in mm");

#endif // FLAGS_HPP