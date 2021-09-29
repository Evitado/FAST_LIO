#pragma once
#include <fstream>

inline void dump_lio_state_to_log(FILE* fp) {
  //  V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
  //  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  //  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2)); // Angle
  //  fprintf(fp,
  //          "%lf %lf %lf ",
  //          state_point.pos(0),
  //          state_point.pos(1),
  //          state_point.pos(2));                // Pos
  //  fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // omega
  //  fprintf(fp,
  //          "%lf %lf %lf ",
  //          state_point.vel(0),
  //          state_point.vel(1),
  //          state_point.vel(2));                // Vel
  //  fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // Acc
  //  fprintf(fp,
  //          "%lf %lf %lf ",
  //          state_point.bg(0),
  //          state_point.bg(1),
  //          state_point.bg(2)); // Bias_g
  //  fprintf(fp,
  //          "%lf %lf %lf ",
  //          state_point.ba(0),
  //          state_point.ba(1),
  //          state_point.ba(2)); // Bias_a
  //  fprintf(fp,
  //          "%lf %lf %lf ",
  //          state_point.grav[0],
  //          state_point.grav[1],
  //          state_point.grav[2]); // Bias_a
  //  fprintf(fp, "\r\n");
  //  fflush(fp);
}
