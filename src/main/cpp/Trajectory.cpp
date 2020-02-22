#include <Trajectory.h>


FollowPath::FollowPath(const std::vector<State>& states) {

}

// void FollowPath::trajectoryInit(std::string jsonFiles[]) {
//     wpi::SmallString<64> deployDirectory;
//     frc::filesystem::GetDeployDirectory(deployDirectory);
//     wpi::sys::path::append(deployDirectory, "paths");
//     wpi::sys::path::append(deployDirectory, "examplePath1.json");
//     frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
// }

// void generateTrajectories(std::string trajectories[]) {
    // for (int i = 0; i < sizeof(trajectories)/sizeof(trajectories[0]) - 1; i++) { // may need to be changed
    //     try {
    //         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectories[i]);
    //         Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //     } catch (std::exception ex) {
    //         DriverStation::ReportError("Unable to open trajectory: " + trajectories[i], ex.getStackTrace());
    //     }
    // }
// }
// Command FollowPath::getAutonomousCommand(string jsonFile) {
//     var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
//                 new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka), drive.getDifferentialDriveKinematics(), 10);

//     TrajectoryConfig config = new TrajectoryConfig(2, 2);

//     config.addConstraint(autoVoltageConstraint);
//     config.setKinematics(drive.getDifferentialDriveKinematics());

//     String trajectoryJSON = jsonFile;

//     try {
//         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//         Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

//         RamseteCommand command = new RamseteCommand(trajectory, drive::getPosition, new RamseteController(2.0, .7),
//                 drive.getFeedFoward(), drive.getDifferentialDriveKinematics(), drive::getWheelSpeeds,
//                 drive.getLeftPIDController(), drive.getRightPIDController(), drive::setVolts, drive);

//         return command.andThen(() -> drive.setVolts(0, 0));
//     } catch (IOException ex) {
//         System.out.println("Unable to open trajectory: " + trajectoryJSON);
//     }
//     // TODO: return empty command to set motors to 0, 0
//     return null;
// }