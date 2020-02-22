#pragma once

#include <frc/trajectory/Trajectory.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <Constants.h>
#include <frc/DriverStation.h>
#include <string>

using namespace frc;
using namespace wpi;

class FollowPath : public Trajectory {
    FollowPath(const std::vector<State>& states);
    // Command getAutonomousCommand(string jsonFile);
    void trajectoryInit();
    void generateTrajectory(std::string trajectories[]);
};