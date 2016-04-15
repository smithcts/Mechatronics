#ifndef SIX_POINT_SENSOR_CAL_H_INCLUDED
#define SIX_POINT_SENSOR_CAL_H_INCLUDED

// Algorithm that can be used to calibrate 3-axis, constant field, sensors such as
// magnetometers and accelerometers.
// field_mag: field magnitude in which the measurements were made e.g. 9.81 for accels
// x, y, z are vectors of six measurements from different orientations
// returns, S[3] and b[3], that are the scale and offset for the axes
// i.e. Measurementx = S[0]*Sensorx + b[0]
bool six_point_sensor_cal(double fiel_mag, const double x[6], const double y[6],
                          const double z[6], double S[3], double b[3]);

#endif
