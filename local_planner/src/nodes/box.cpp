#include "box.h"

Box::Box(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
    : xmin { x_min },
      xmax { x_max },
      ymin { y_min },
      ymax { y_max },
      zmin { z_min },
      zmax { z_max }{
}

Box::Box()
    : xmin { 0 },
      xmax { 0 },
      ymin { 0 },
      ymax { 0 },
      zmin { 0 },
      zmax { 0 } {
}

Box::~Box() {
}

