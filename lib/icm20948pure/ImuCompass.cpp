#include "ImuCompass.h"
#include <math.h>

namespace ImuCompass
{
    double rawHeadingDeg(const Vec3 &mag, double rollRad, double pitchRad)
    {
        double Xh = mag.x * cos(pitchRad) + mag.y * sin(rollRad) * sin(pitchRad) + mag.z * cos(rollRad) * sin(pitchRad);
        double Yh = mag.y * cos(rollRad) - mag.z * sin(rollRad);
        return atan2(Yh, Xh) * (180.0 / M_PI);
    }
}
