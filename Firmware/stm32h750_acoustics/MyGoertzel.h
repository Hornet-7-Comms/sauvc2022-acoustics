// Adapted from https://github.com/jacobrosenthal/Goertzel
#pragma once

float Goertzel_coefficient(float targetFrequency, float samplingFrequency) {
  float omega = (2.0 * PI * targetFrequency) / samplingFrequency;
  float coeff = 2.0 * cos(omega);
  return coeff;
}

inline float Goertzel_magnitudeSquare(uint16_t *, uint16_t, float) __attribute__((always_inline));
inline float Goertzel_magnitudeSquare(uint16_t * sampleArray, uint16_t sampleSize, float coeff) {
  float Q0;
  float Q1 = 0;
  float Q2 = 0;

  /* Process the samples. */
  for (uint16_t index = 0; index < sampleSize; index++) {
    Q0 = coeff*Q1 - Q2 + ((float)sampleArray[index]);
    Q2 = Q1;
    Q1 = Q0;
  }

  /* Formula for optimised Goertzel algorithm */
  //return sqrtf(Q1*Q1 + Q2*Q2 - coeff*Q1*Q2); // ARM-optimised floating point square root

  return (Q1*Q1 + Q2*Q2 - coeff*Q1*Q2);
}
