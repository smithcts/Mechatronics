#ifndef MATH_UTIL_H_INCLUDED
#define MATH_UTIL_H_INCLUDED

#include "physical_constants.h"

// Limit 'input' parameter to fall in between lower limit and upper limit.
template <class T>
T limit(T input, T lower_limit, T upper_limit)
{
    if (input < lower_limit) {  return lower_limit;  }
    if (input > upper_limit) {  return upper_limit;  }
    return input;
}

// Wrap angle (in radians) to be between (-PI, PI]
template <class T>
T wrap_angle(T angle)
{
    while (angle <= -PI) {  angle += 2*PI;  }
    while (angle  >  PI) {  angle -= 2*PI;  }
    return angle;
}

// Return the greater (>) of the two parameters.
template <class T>
T max(T p1, T p2)
{
    return (p1 > p2) ? p1 : p2;
}

// Return the lesser (<) of the two parameters.
template <class T>
T min(T p1, T p2)
{
    return (p1 < p2) ? p1 : p2;
}

// Return element in specified array with the largest value.
template <class T>
T max(T * array, uint32_t num_elements)
{
    if (num_elements == 0) { return 0; }
    T max = array[0];
    for (uint32_t i = 1; i < num_elements; i++)
    {
        if (array[i] > max)
        {
            max = array[i];
        }
    }
    return max;
}

// Return element in specified array with the smallest value.
template <class T>
T min(T * array, uint32_t num_elements)
{
    if (num_elements == 0) { return 0; }
    T min = array[0];
    for (uint32_t i = 1; i < num_elements; i++)
    {
        if (array[i] < min)
        {
            min = array[i];
        }
    }
    return min;
}

// Return average of all elements in array.
template <class T>
T average(T * array, uint32_t num_elements)
{
    T sum = 0;
    for (uint32_t i = 0; i < num_elements; i++)
    {
        sum += array[i];
    }

    return sum / num_elements;
}

#endif
