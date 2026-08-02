#include "ImuDeviationTable.h"
#include "ImuAngleMath.h"
#include <math.h>

void DeviationTable::clear()
{
    count = 0;
}

DeviationEntry DeviationTable::entryAt(int index) const
{
    if (index < 0 || index >= count)
        return DeviationEntry();
    return entries[index];
}

int DeviationTable::findNearestIndex(double measuredHeadingDeg) const
{
    if (count == 0)
        return -1;
    int best = 0;
    double bestDist = fabs(ImuAngleMath::shortestDiff(entries[0].measuredHeadingDeg, measuredHeadingDeg));
    for (int i = 1; i < count; i++)
    {
        double d = fabs(ImuAngleMath::shortestDiff(entries[i].measuredHeadingDeg, measuredHeadingDeg));
        if (d < bestDist)
        {
            bestDist = d;
            best = i;
        }
    }
    return best;
}

void DeviationTable::insertSorted(const DeviationEntry &e)
{
    int pos = count;
    for (int i = 0; i < count; i++)
    {
        if (e.measuredHeadingDeg < entries[i].measuredHeadingDeg)
        {
            pos = i;
            break;
        }
    }
    for (int i = count; i > pos; i--)
        entries[i] = entries[i - 1];
    entries[pos] = e;
    count++;
}

bool DeviationTable::addEntry(double measuredHeadingDeg, double referenceHeadingDeg)
{
    measuredHeadingDeg = ImuAngleMath::wrap360(measuredHeadingDeg);
    double correction = ImuAngleMath::shortestDiff(measuredHeadingDeg, referenceHeadingDeg);

    int nearest = findNearestIndex(measuredHeadingDeg);
    if (nearest >= 0 && fabs(ImuAngleMath::shortestDiff(entries[nearest].measuredHeadingDeg, measuredHeadingDeg)) < 1.0)
    {
        entries[nearest].measuredHeadingDeg = measuredHeadingDeg;
        entries[nearest].correctionDeg = correction;
        return true;
    }

    if (count >= MAX_ENTRIES)
        return false;

    DeviationEntry e;
    e.measuredHeadingDeg = measuredHeadingDeg;
    e.correctionDeg = correction;
    insertSorted(e);
    return true;
}

double DeviationTable::correctionDegFor(double measuredHeadingDeg) const
{
    if (count == 0)
        return 0.0;
    if (count == 1)
        return entries[0].correctionDeg;

    double h = ImuAngleMath::wrap360(measuredHeadingDeg);

    int lower = -1;
    for (int i = 0; i < count; i++)
    {
        if (entries[i].measuredHeadingDeg <= h)
            lower = i;
    }

    int a, b;
    if (lower < 0)
    {
        // h is before the first entry - bracket wraps from the last
        // entry to the first.
        a = count - 1;
        b = 0;
    }
    else if (lower == count - 1)
    {
        // h is at or after the last entry - bracket wraps to the first.
        a = count - 1;
        b = 0;
    }
    else
    {
        a = lower;
        b = lower + 1;
    }

    double hA = entries[a].measuredHeadingDeg;
    double hB = entries[b].measuredHeadingDeg;
    double distAtoH = fmod(h - hA + 360.0, 360.0);
    double distAtoB = fmod(hB - hA + 360.0, 360.0);
    double t = (distAtoB > 1e-9) ? (distAtoH / distAtoB) : 0.0;

    double corrA = entries[a].correctionDeg;
    double corrB = entries[b].correctionDeg;
    return corrA + ImuAngleMath::shortestDiff(corrA, corrB) * t;
}

double DeviationTable::apply(double measuredHeadingDeg) const
{
    return ImuAngleMath::wrap360(measuredHeadingDeg + correctionDegFor(measuredHeadingDeg));
}
