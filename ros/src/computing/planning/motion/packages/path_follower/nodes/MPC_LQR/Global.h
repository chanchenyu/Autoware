#ifndef DGC_GLOBAL_H_
#define DGC_GLOBAL_H_

#include <sys/stat.h>
#include <cmath>


namespace dgc {

inline double dgc_mph2ms(double mph) {
    return (mph * 0.44704);
}

}// namespace dgc
#endif