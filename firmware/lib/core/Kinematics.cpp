#include "Kinematics.h"

#include <algorithm>
#include <cmath>

const LegGeometry kLegs[6] = {
    {"RF", 100.94f, -57.11f},
    {"RM", 0.0f, -105.72f},
    {"RR", -100.94f, -57.11f},
    {"LR", -100.94f, 57.11f},
    {"LM", 0.0f, 105.72f},
    {"LF", 100.94f, 57.11f},
};

float legMountAngleRad(const LegGeometry& leg) {
    return std::atan2(leg.originYMm, leg.originXMm);
}

JointAngles clampJointAngles(const JointAngles& angles) { return clampJointAnglesWithClip(angles).angles; }

namespace {
float clampWithOvershoot(float value, float lo, float hi, float& overshootOut) {
    if (value > hi) {
        overshootOut = value - hi;
        return hi;
    }
    if (value < lo) {
        overshootOut = lo - value;
        return lo;
    }
    overshootOut = 0.0f;
    return value;
}
}  // namespace

ClampResult clampJointAnglesWithClip(const JointAngles& angles) {
    ClampResult out;
    float coxaOver, femurOver, tibiaOver;
    out.angles.coxaDeg = clampWithOvershoot(angles.coxaDeg, kCoxaMinDeg, kCoxaMaxDeg, coxaOver);
    out.angles.femurDeg = clampWithOvershoot(angles.femurDeg, kFemurMinDeg, kFemurMaxDeg, femurOver);
    out.angles.tibiaDeg = clampWithOvershoot(angles.tibiaDeg, kTibiaMinDeg, kTibiaMaxDeg, tibiaOver);
    out.worstOvershootDeg = std::max({coxaOver, femurOver, tibiaOver});
    return out;
}

Point3 forwardKinematics(const JointAngles& angles, const LegGeometry& leg, float originZMm) {
    const float mountAngleRad = legMountAngleRad(leg);
    const float alpha = angles.coxaDeg * (M_PI / 180.0f) + mountAngleRad;
    const float beta = angles.femurDeg * (M_PI / 180.0f);
    const float gamma = angles.tibiaDeg * (M_PI / 180.0f);

    const float ca = std::cos(alpha), sa = std::sin(alpha);
    const float cb = std::cos(beta), sb = std::sin(beta);
    const float cbg = std::cos(beta + gamma), sbg = std::sin(beta + gamma);

    const float coxaJointX = leg.originXMm + kCoxaLengthMm * ca;
    const float coxaJointY = leg.originYMm + kCoxaLengthMm * sa;

    Point3 foot;
    foot.x = coxaJointX + (kFemurLengthMm * cb + kTibiaLengthMm * cbg) * ca;
    foot.y = coxaJointY + (kFemurLengthMm * cb + kTibiaLengthMm * cbg) * sa;
    foot.z = originZMm + (kFemurLengthMm * sb + kTibiaLengthMm * sbg);
    return foot;
}

JointAngles inverseKinematics(const Point3& target, const LegGeometry& leg, float originZMm) {
    return inverseKinematicsWithClip(target, leg, originZMm).angles;
}

IkResult inverseKinematicsWithClip(const Point3& target, const LegGeometry& leg, float originZMm) {
    const float mountAngleRad = legMountAngleRad(leg);
    const float relativeX = target.x - leg.originXMm;
    const float relativeY = target.y - leg.originYMm;
    const float relativeZ = target.z - originZMm;

    const float c = std::cos(-mountAngleRad);
    const float s = std::sin(-mountAngleRad);
    const float localX = relativeX * c - relativeY * s;
    const float localY = relativeX * s + relativeY * c;

    const float lXy = std::hypot(localX, localY);
    const float lForward = lXy - kCoxaLengthMm;
    float d = std::hypot(lForward, relativeZ);

    const float dMax = kFemurLengthMm + kTibiaLengthMm;
    const float dMin = std::fabs(kFemurLengthMm - kTibiaLengthMm);
    float overshootMm = 0.0f;
    if (d > dMax) {
        overshootMm = d - dMax;
        d = dMax - 0.001f;
    }
    if (d < dMin) {
        overshootMm = dMin - d;
        d = dMin + 0.001f;
    }

    const float gammaRaw =
        std::acos((kFemurLengthMm * kFemurLengthMm + kTibiaLengthMm * kTibiaLengthMm - d * d) /
                  (2.0f * kFemurLengthMm * kTibiaLengthMm)) -
        static_cast<float>(M_PI);  // provably in [-pi, 0] for any clamped d

    const float beta1 = std::atan2(relativeZ, lForward);
    const float beta2 = std::acos((kFemurLengthMm * kFemurLengthMm + d * d - kTibiaLengthMm * kTibiaLengthMm) /
                                   (2.0f * kFemurLengthMm * d));
    const float beta = beta1 + beta2;

    IkResult out;
    out.angles.coxaDeg = std::atan2(localY, localX) * (180.0f / M_PI);
    out.angles.femurDeg = beta * (180.0f / M_PI);
    out.angles.tibiaDeg = gammaRaw * (180.0f / M_PI);  // raw gamma -- see toServoDeg() for -gamma
    out.dOvershootMm = overshootMm;
    return out;
}

ServoDeg toServoDeg(const JointAngles& angles) {
    ServoDeg out;
    out.coxaDeg = 90.0f + angles.coxaDeg;
    out.femurDeg = 90.0f + angles.femurDeg;
    out.tibiaDeg = -angles.tibiaDeg;
    return out;
}
