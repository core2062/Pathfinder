
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/units.h>
#include <wpi/math>

#pragma once

namespace DriveConstants {
constexpr int kLeftMotor1Port = 10;
constexpr int kLeftMotor2Port = 11;
constexpr int kRightMotor1Port = 12;
constexpr int kRightMotor2Port = 13;

constexpr int kLeftEncoderPorts[]{0, 1};
constexpr int kRightEncoderPorts[]{2, 3};
constexpr bool kLeftEncoderReversed = false;
constexpr bool kRightEncoderReversed = true;

constexpr auto kTrackwidth = 0.7747_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr int kEncoderCPR = 1024;
constexpr double kWheelDiameterInches = 6;
constexpr double kEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterInches * wpi::math::pi) / static_cast<double>(kEncoderCPR);

constexpr bool kGyroReversed = true;

constexpr auto ks = 1.88_V;
constexpr auto kv = 0.225 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.129 * 1_V * 1_s * 1_s / 1_m;

constexpr double kPDriveVel = 6.13;
}  // namespace DriveConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;

// Reasonable baseline values for a RAMSETE follower in units of meters and
// seconds
constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
