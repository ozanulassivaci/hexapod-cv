#include "Gait.h"

#include <cmath>

const JointAngles kNeutralStanceAngles = {0.0f, 45.0f, -90.0f};

namespace {
float clampf(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

float slewOne(float current, float goal, float maxDelta) {
    float delta = goal - current;
    if (delta > maxDelta) delta = maxDelta;
    if (delta < -maxDelta) delta = -maxDelta;
    return current + delta;
}
}  // namespace

float homeZMm(float bodyHeight) {
    const float frac = clampf(bodyHeight, 0.0f, 100.0f) / 100.0f;
    return kBodyHeightZCrouchedMm + frac * (kBodyHeightZTallMm - kBodyHeightZCrouchedMm);
}

Point3 homePosition(const LegGeometry& leg, float bodyHeight) {
    Point3 point = forwardKinematics(kNeutralStanceAngles, leg);
    point.z = homeZMm(bodyHeight);
    return point;
}

bool isInStance(int legIndex, float phase) {
    float localPhase = std::fmod(phase, 1.0f);
    if (localPhase < 0.0f) localPhase += 1.0f;
    if (legIndex % 2 != 0) {
        localPhase = std::fmod(localPhase + 0.5f, 1.0f);
    }
    return localPhase < 0.5f;
}

bool isMotionIdle(float vx, float vy, float speed, float rotation) {
    const float speedFrac = clampf(speed, 0.0f, 100.0f) / 100.0f;
    const float magnitude = std::hypot(vx, vy);
    if (speedFrac <= 1e-9f) return true;
    return magnitude <= 1e-9f && std::fabs(rotation) <= 1e-9f;
}

Point3 footTarget(int legIndex, const LegGeometry& leg, float phase, float vx, float vy, float speed,
                   float rotation, float bodyHeight) {
    Point3 home = homePosition(leg, bodyHeight);

    const float speedFrac = clampf(speed, 0.0f, 100.0f) / 100.0f;
    float magnitude = std::hypot(vx, vy);
    if (magnitude > 1.0f) {
        vx /= magnitude;
        vy /= magnitude;
    }

    if (isMotionIdle(vx, vy, speed, rotation)) {
        return home;
    }

    float localPhase = std::fmod(phase, 1.0f);
    if (localPhase < 0.0f) localPhase += 1.0f;
    if (legIndex % 2 != 0) {
        localPhase = std::fmod(localPhase + 0.5f, 1.0f);
    }

    float stepScale, stepZ;
    if (localPhase < 0.5f) {
        const float phaseNorm = localPhase * 2.0f;
        stepScale = 0.5f - phaseNorm;
        stepZ = 0.0f;
    } else {
        const float phaseNorm = (localPhase - 0.5f) * 2.0f;
        stepScale = phaseNorm - 0.5f;
        stepZ = std::sin(phaseNorm * static_cast<float>(M_PI)) * kStepHeightMm;
    }

    const float stepX = vx * speedFrac * stepScale * kStepLengthMm;
    const float stepY = vy * speedFrac * stepScale * kStepLengthMm;
    const float stepRotRad = rotation * speedFrac * stepScale * kRotationStepRad;

    const float cRot = std::cos(stepRotRad), sRot = std::sin(stepRotRad);
    const float rotatedX = home.x * cRot - home.y * sRot;
    const float rotatedY = home.x * sRot + home.y * cRot;

    return Point3{rotatedX + stepX, rotatedY + stepY, home.z + stepZ};
}

GaitState GaitState::initial(float bodyHeight) {
    GaitState state;
    state.phase = 0.0f;
    state.bodyHeight = bodyHeight;
    for (int i = 0; i < 6; ++i) {
        Point3 home = homePosition(kLegs[i], bodyHeight);
        state.legAngles[i] = clampJointAngles(inverseKinematics(home, kLegs[i]));
    }
    return state;
}

void GaitState::footPositions(Point3 outPositions[6]) const {
    for (int i = 0; i < 6; ++i) {
        outPositions[i] = forwardKinematics(legAngles[i], kLegs[i]);
    }
}

GaitState stepGait(const GaitState& state, float dtS, float vx, float vy, float speed, float rotation,
                    float bodyHeight) {
    const float speedFrac = clampf(speed, 0.0f, 100.0f) / 100.0f;
    float newPhase = std::fmod(state.phase + kBasePhaseRateHz * speedFrac * dtS, 1.0f);
    if (newPhase < 0.0f) newPhase += 1.0f;

    const float maxDeltaDeg = kMaxSlewDegPerS * (dtS > 0.0f ? dtS : 0.0f);

    GaitState next;
    next.phase = newPhase;
    next.bodyHeight = bodyHeight;
    for (int i = 0; i < 6; ++i) {
        Point3 goalPoint = footTarget(i, kLegs[i], newPhase, vx, vy, speed, rotation, bodyHeight);
        JointAngles goalAngles = clampJointAngles(inverseKinematics(goalPoint, kLegs[i]));
        const JointAngles& current = state.legAngles[i];
        next.legAngles[i] = JointAngles{
            slewOne(current.coxaDeg, goalAngles.coxaDeg, maxDeltaDeg),
            slewOne(current.femurDeg, goalAngles.femurDeg, maxDeltaDeg),
            slewOne(current.tibiaDeg, goalAngles.tibiaDeg, maxDeltaDeg),
        };
    }
    return next;
}
