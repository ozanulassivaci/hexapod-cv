// GENERATED FILE -- do not edit by hand.
//
// Source: transport/constants.yaml
// Regenerate with: python scripts/gen_protocol_constants.py (or: make gen-protocol)
//
// This is a manual step, not a build hook -- see docs/protocol.md Section 5
// for why. Because it's manual, it can be forgotten; the runtime cross-check
// in transport/link.py (RobotLink.constants_warning) and its firmware-side
// counterpart (telemetry's link_timeout_s field) are what actually catch
// drift, by comparing the two sides' compiled-in values at connect time.
// This file being current is a convenience, not the safety property.

#pragma once

#define HX_PROTOCOL_VERSION 1
#define HX_SEQUENCE_BITS 32
#define HX_SEQUENCE_MODULUS (1ULL << HX_SEQUENCE_BITS)
#define HX_LINK_TIMEOUT_S 1.0f
#define HX_HEARTBEAT_INTERVAL_S 0.1f
#define HX_CALIBRATION_ARM_TIMEOUT_S 30.0f
#define HX_BENCH_ARM_TIMEOUT_S 60.0f
