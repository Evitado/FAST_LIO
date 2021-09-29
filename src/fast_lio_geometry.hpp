#pragma once
#include "LaserMapper.hpp"

void pointBodyToWorld_ikfom(PointType const* const pi,
                            PointType* const po,
                            state_ikfom& s) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const* const pi, PointType* const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(
      state_point.rot *
          (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
      state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1>& pi, Matrix<T, 3, 1>& po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(
      state_point.rot *
          (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
      state_point.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const* const pi, PointType* const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(
      state_point.rot *
          (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
      state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const* const pi, PointType* const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar +
                 state_point.offset_T_L_I);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

template <typename T>
void set_posestamp(T& out) {
  out.pose.position.x = state_point.pos(0);
  out.pose.position.y = state_point.pos(1);
  out.pose.position.z = state_point.pos(2);
  out.pose.orientation.x = geoQuat.x;
  out.pose.orientation.y = geoQuat.y;
  out.pose.orientation.z = geoQuat.z;
  out.pose.orientation.w = geoQuat.w;
}
