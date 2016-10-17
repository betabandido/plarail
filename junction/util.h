#ifndef UTIL_H
#define UTIL_H

#undef abs

template<typename T>
static inline T abs(T x) {
  if (x < 0)
    return -x;
  return x;
}

#endif // UTIL_H

