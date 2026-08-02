#include "ImuMagCal2D.h"
#include <math.h>

namespace
{
    int clampSectorCount(int n)
    {
        if (n < 1)
            return 1;
        if (n > ImuMagCal2D::MAX_SECTORS)
            return ImuMagCal2D::MAX_SECTORS;
        return n;
    }

    double clamp01(double v)
    {
        if (v < 0)
            return 0;
        if (v > 1)
            return 1;
        return v;
    }
}

void ImuMagCal2D::start()
{
    st = MagCal2DState::Collecting;
    count = 0;
    sumX = sumY = sumXX = sumYY = sumXY = 0;
    sumXR2 = sumYR2 = sumR2 = 0;
    for (int i = 0; i < MAX_SECTORS; i++)
        sectorVisited[i] = false;
    failReason = "";
}

void ImuMagCal2D::cancel()
{
    st = MagCal2DState::Idle;
    count = 0;
}

void ImuMagCal2D::fail(const char *reason)
{
    st = MagCal2DState::Failed;
    failReason = reason;
}

bool ImuMagCal2D::addSample(double x, double y, const MagCal2DConfig &cfg)
{
    if (st != MagCal2DState::Collecting)
        return false;

    double r = sqrt(x * x + y * y);
    if (r < cfg.minFieldMagnitudeUT || r > cfg.maxFieldMagnitudeUT)
        return false;

    count++;
    sumX += x;
    sumY += y;
    sumXX += x * x;
    sumYY += y * y;
    sumXY += x * y;
    double r2 = x * x + y * y;
    sumXR2 += x * r2;
    sumYR2 += y * r2;
    sumR2 += r2;

    int sectorCount = clampSectorCount(cfg.sectorCount);
    double angleDeg = atan2(y, x) * 180.0 / M_PI;
    if (angleDeg < 0)
        angleDeg += 360.0;
    int sector = (int)(angleDeg / (360.0 / sectorCount));
    if (sector < 0)
        sector = 0;
    if (sector >= sectorCount)
        sector = sectorCount - 1;
    sectorVisited[sector] = true;

    return true;
}

double ImuMagCal2D::coverageFraction(const MagCal2DConfig &cfg) const
{
    int sectorCount = clampSectorCount(cfg.sectorCount);
    int visited = 0;
    for (int i = 0; i < sectorCount; i++)
        if (sectorVisited[i])
            visited++;
    return (double)visited / (double)sectorCount;
}

double ImuMagCal2D::spanDeg(const MagCal2DConfig &cfg) const
{
    int sectorCount = clampSectorCount(cfg.sectorCount);
    double degPerSector = 360.0 / sectorCount;

    bool anyVisited = false;
    for (int i = 0; i < sectorCount; i++)
        if (sectorVisited[i])
        {
            anyVisited = true;
            break;
        }
    if (!anyVisited)
        return 0.0;

    bool allVisited = true;
    for (int i = 0; i < sectorCount; i++)
        if (!sectorVisited[i])
        {
            allVisited = false;
            break;
        }
    if (allVisited)
        return 360.0;

    // Largest circular run of unvisited sectors: walk the array twice
    // around (2*sectorCount) tracking the current run length, capped at
    // sectorCount so it can't overcount past a full lap.
    int maxGap = 0, currentGap = 0;
    for (int i = 0; i < sectorCount * 2; i++)
    {
        if (!sectorVisited[i % sectorCount])
        {
            currentGap++;
            if (currentGap > maxGap)
                maxGap = currentGap;
            if (maxGap > sectorCount)
                maxGap = sectorCount;
        }
        else
        {
            currentGap = 0;
        }
    }

    return 360.0 - (double)maxGap * degPerSector;
}

bool ImuMagCal2D::stop(const MagCal2DConfig &cfg)
{
    if (st != MagCal2DState::Collecting)
        return false;

    if (count < cfg.minSamples)
    {
        fail("insufficient samples - swing longer");
        return false;
    }

    double coverage = coverageFraction(cfg);
    if (coverage < cfg.minCoverageFraction)
    {
        fail("insufficient sector coverage - swing through more headings");
        return false;
    }

    // Kasa's algebraic circle fit: x^2+y^2 = D*x + E*y + F, linear least
    // squares via the normal equations below, solved with Cramer's rule
    // (a fixed 3x3 system, not worth pulling in a general matrix solver
    // for).
    double n = (double)count;
    double m[3][3] = {
        {sumXX, sumXY, sumX},
        {sumXY, sumYY, sumY},
        {sumX, sumY, n},
    };
    double rhs[3] = {sumXR2, sumYR2, sumR2};

    double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                 m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                 m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (!isfinite(det) || fabs(det) < 1e-9)
    {
        fail("fit numerically degenerate - samples too clustered along a line");
        return false;
    }

    auto solveFor = [&](int col) {
        double mm[3][3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                mm[r][c] = (c == col) ? rhs[r] : m[r][c];
        return (mm[0][0] * (mm[1][1] * mm[2][2] - mm[1][2] * mm[2][1]) -
                mm[0][1] * (mm[1][0] * mm[2][2] - mm[1][2] * mm[2][0]) +
                mm[0][2] * (mm[1][0] * mm[2][1] - mm[1][1] * mm[2][0])) /
               det;
    };
    double D = solveFor(0), E = solveFor(1), F = solveFor(2);

    double a = D / 2.0, b = E / 2.0;
    double r2 = F + a * a + b * b;
    if (!isfinite(r2) || r2 <= 0)
    {
        fail("fit produced an invalid radius");
        return false;
    }

    // Mean squared radial distance from the fitted center, computed from
    // already-accumulated moments (no stored samples needed):
    // sum((x-a)^2+(y-b)^2) = sumXX+sumYY - 2a*sumX - 2b*sumY + n*(a^2+b^2)
    double meanDist2 = (sumXX + sumYY - 2 * a * sumX - 2 * b * sumY + n * (a * a + b * b)) / n;
    double residualFraction = fabs(meanDist2 - r2) / r2;
    if (residualFraction > cfg.maxResidualFraction)
    {
        fail("fit residual too large - data may be noisy or not a clean circle");
        return false;
    }

    // Per-axis scale: separately estimate each axis's RMS spread around
    // the fitted center, then scale both axes to a common reference (their
    // geometric mean) - an axis-aligned approximation of soft-iron
    // correction. Full off-axis (rotated) soft-iron correction is what
    // the offline 3D ellipsoid tool is for.
    double varX = (sumXX - 2 * a * sumX + n * a * a) / n;
    double varY = (sumYY - 2 * b * sumY + n * b * b) / n;
    double rx = sqrt(varX > 0 ? varX : 0);
    double ry = sqrt(varY > 0 ? varY : 0);
    double refR = sqrt(rx * ry);
    if (rx > 1e-6 && ry > 1e-6 && refR > 1e-6)
    {
        scaleX = refR / rx;
        scaleY = refR / ry;
    }
    else
    {
        scaleX = 1.0;
        scaleY = 1.0;
    }

    biasX = a;
    biasY = b;
    quality = clamp01(coverage) * clamp01(1.0 - residualFraction / cfg.maxResidualFraction);
    failReason = "";
    st = MagCal2DState::Ready;
    return true;
}
