package frc.robot.commands.Drive;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.testingdashboard.Command;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;
import frc.robot.OI;

public class PathToPose extends Command {

    Supplier<Pose2d> m_targetPose;
    Pose2d m_currentTarget;
    edu.wpi.first.wpilibj2.command.Command m_currentPathCommand;
    PathPlannerPath currentPath;
    Drive m_Drive;
  
    public PathToPose(Supplier<Pose2d> poseSupplier) {
        super (Drive.getInstance(), "Drive", "Path to Pose");
        m_Drive = Drive.getInstance();
        m_targetPose = poseSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Drive.getInstance());
    }

    @Override
    public void initialize(){
        Pose2d destination =  m_targetPose.get();
        var rotations = new LinkedList<RotationTarget>();
        var waypoints = new LinkedList<Waypoint>();
        var currentRotation = Drive.getInstance().m_field.getRobotPose().getRotation();
        var currentLocation = Drive.getInstance().m_field.getRobotPose().getTranslation();
        var currentControlPoint = currentLocation.plus(new Translation2d(Meters.of( 0.25 * currentRotation.getCos()), Meters.of( 0.25 * currentRotation.getSin())));
        waypoints.add(new Waypoint(null, currentLocation, currentControlPoint));
        var approach = destination.getTranslation();
        switch (OI.getInstance().getApproachType()) {
            case c_LeftApproach:
                approach.plus(new Translation2d(Meters.of( -1 * destination.getRotation().getSin()),Meters.of(destination.getRotation().getCos())));
                waypoints.add(new Waypoint(approach, destination.getTranslation(), null));
                break;
            case c_RightApproach:
                approach.plus(new Translation2d(Meters.of(destination.getRotation().getSin()),Meters.of( -1 * destination.getRotation().getCos())));
                waypoints.add(new Waypoint(approach, destination.getTranslation(), null));
                break;
            case c_LeftSpin:
                rotations.add(new RotationTarget(0.90, new Rotation2d(Units.degreesToRadians(90))));
                waypoints.add(new Waypoint(approach, destination.getTranslation(), null));
                break;
            case c_RightSpin:
                rotations.add(new RotationTarget(0.90, new Rotation2d(Units.degreesToRadians(-90))));
                waypoints.add(new Waypoint(approach, destination.getTranslation(), null));
                break;
            case c_Straight:
                default:
                approach.minus(new Translation2d(Meters.of(destination.getRotation().getCos()),Meters.of(destination.getRotation().getSin())));
                waypoints.add(new Waypoint(approach, destination.getTranslation(), null));
                break;
        }

        // Create the path using the waypoints created above
        currentPath = new PathPlannerPath(
            waypoints,
            rotations,
            new ArrayList<PointTowardsZone>(), //PointTowardsZones
            new ArrayList<ConstraintsZone>(), // ConstraintZones
            new ArrayList<EventMarker>(), // EvevntMarkers
            new PathConstraints(Constants.kAutoMaxSpeedMpS * 10,
                Constants.kAutoMaxAccelerationMpSS * 10,
                Constants.kAutoMaxAngularSpeedRpS * 10,
                Constants.kAutoMaxAngularAccelRpSS * 10),
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, destination.getRotation()),
            false
        );

        // Prevent the path from being flipped if the coordinates are already correct
        currentPath.preventFlipping = true;
    }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_currentPathCommand == null || m_currentTarget == null || !m_currentTarget.equals(m_targetPose))
    {
      m_currentPathCommand = AutoBuilder.followPath(currentPath);
      m_currentPathCommand.initialize();
    }
    m_currentPathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_currentPathCommand != null) {
      m_currentPathCommand.end(interrupted);
      m_currentPathCommand = null;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_currentPathCommand != null) {
      return m_currentPathCommand.isFinished();
    }
    return true;
  }
}
