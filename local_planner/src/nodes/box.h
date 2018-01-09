class Box
{
 public:
  Box();
  Box(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
  ~Box();

  double xmin;
  double xmax;
  double ymin;
  double ymax;
  double zmin;
  double zmax;
};
