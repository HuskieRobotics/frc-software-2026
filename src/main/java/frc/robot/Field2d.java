package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;

/**
 * This singleton class models the field as a collection of regions. This class is used to create a
 * path from a starting pose in one region to an ending pose in another region that passes through
 * the transition points defined for those regions.
 *
 * <p>The coordinate system of the field is oriented such that the origin is in the lower left
 * corner when the blue alliance is to the left (i.e., to the blue alliance driver's right).
 */
public class Field2d {
  private static Field2d instance = null;

  private Region2d[] regions;

  // for all four possible banks depending on alliance
  private Pose2d[] banks = new Pose2d[4];

  private Alliance alliance = DriverStation.Alliance.Blue;

  private Region2d transformedAllianceZone;
  private Region2d transformedLeftTrenchZoneBLUE;
  private Region2d transformedRightTrenchZoneBLUE;
  private Region2d transformedLeftTrenchZoneRED;
  private Region2d transformedRightTrenchZoneRED;
  private Region2d transformedLeftBumpZoneBLUE;
  private Region2d transformedRightBumpZoneBLUE;
  private Region2d transformedLeftBumpZoneRED;
  private Region2d transformedRightBumpZoneRED;
  private final double ALLIANCE_ZONE_BUFFER_INCHES = 10;

  private final double TRENCH_ZONE_BUFFER_X_INCHES = 7;

  private final double BUMP_ZONE_BUFFER_X_INCHES = 7;

  private final double BANK_BUFFER_FROM_TRENCH_INCHES = 12;

  /**
   * Get the singleton instance of the Field2d class.
   *
   * @return the singleton instance of the Field2d class
   */
  public static Field2d getInstance() {
    if (instance == null) {
      instance = new Field2d();
    }
    return instance;
  }
  /**
   * Construct a Field2d from an array of regions. These regions should not be overlapping (aside
   * from edges) and any regions with overlapping edges should be neighbors (see
   * Region2d::addNeighbor).
   *
   * @param regions the regions that define the field
   */
  public void setRegions(Region2d[] regions) {
    this.regions = regions;
  }

  public void populateAllianceZone() {

    // since positive x is defined at forward if we move the far side x back 2 inches it should
    // result in giving us a 2 inch buffer
    double bufferAZ = Units.inchesToMeters(ALLIANCE_ZONE_BUFFER_INCHES);
    double safeFarSideX = FieldConstants.LinesVertical.allianceZone - bufferAZ;

    Translation2d[] zoneCorners =
        new Translation2d[] {
          // bottom right corner
          new Translation2d(0.0, 0.0),

          // top right corner
          new Translation2d(safeFarSideX, 0.0),

          // top left corner
          new Translation2d(safeFarSideX, FieldConstants.fieldWidth),

          // bottom left corner
          new Translation2d(0.0, FieldConstants.fieldWidth)
        };

    this.transformedAllianceZone = new Region2d(zoneCorners);
  }

  /**
   * For now, this method populates all four banks (although only two can be used for any specific
   * match) In the future, it can be improved to only have two banks constructed at the start of the
   * match based on alliance.
   *
   * <p>The bank will have a temp target-relative y-tolerance of 0.25 meters = ~10 inches
   * (field-relative x-tolerance) We must be at least 10 inches outside of trench zone Target
   * rotations of 90 degrees on the right, 270 on the left for the back of robot to be facing the
   * wall
   */
  public void populateBanks() {
    // blue left bank
    banks[0] =
        new Pose2d(
            FieldConstants.LinesVertical.allianceZone
                - Units.inchesToMeters(TRENCH_ZONE_BUFFER_X_INCHES)
                - Units.inchesToMeters(BANK_BUFFER_FROM_TRENCH_INCHES),
            FieldConstants.fieldWidth
                - RobotConfig.getInstance().getRobotWidthWithBumpers().in(Meters) / 2.0,
            Rotation2d.fromDegrees(270));

    // blue right bank
    banks[1] =
        new Pose2d(
            FieldConstants.LinesVertical.allianceZone
                - Units.inchesToMeters(TRENCH_ZONE_BUFFER_X_INCHES)
                - Units.inchesToMeters(BANK_BUFFER_FROM_TRENCH_INCHES),
            RobotConfig.getInstance().getRobotWidthWithBumpers().in(Meters) / 2.0,
            Rotation2d.fromDegrees(90));

    // red left bank
    banks[2] = FlippingUtil.flipFieldPose(banks[0]);

    // red right bank
    banks[3] = FlippingUtil.flipFieldPose(banks[1]);
  }

  public void populateTrenchZone() {

    double bufferTrenchX = Units.inchesToMeters(TRENCH_ZONE_BUFFER_X_INCHES);

    Translation2d[] leftTrenchEdgesBLUE =
        new Translation2d[] {

          // Left Trench
          new Translation2d(
              FieldConstants.LinesVertical.allianceZone - bufferTrenchX,
              FieldConstants.LinesHorizontal.leftTrenchOpenStart),
          new Translation2d(
              FieldConstants.LinesVertical.allianceZone - bufferTrenchX,
              FieldConstants.LinesHorizontal.leftTrenchOpenEnd),
          new Translation2d(
              FieldConstants.LinesVertical.neutralZoneNear + bufferTrenchX,
              FieldConstants.LinesHorizontal.leftTrenchOpenEnd),
          new Translation2d(
              FieldConstants.LinesVertical.neutralZoneNear + bufferTrenchX,
              FieldConstants.LinesHorizontal.leftTrenchOpenStart),
        };

    Translation2d[] rightTrenchEdgesBLUE =
        new Translation2d[] {
          // Right Trench
          new Translation2d(
              FieldConstants.LinesVertical.allianceZone - bufferTrenchX,
              FieldConstants.LinesHorizontal.rightTrenchOpenEnd),
          new Translation2d(
              FieldConstants.LinesVertical.allianceZone - bufferTrenchX,
              FieldConstants.LinesHorizontal.rightTrenchOpenStart),
          new Translation2d(
              FieldConstants.LinesVertical.neutralZoneNear + bufferTrenchX,
              FieldConstants.LinesHorizontal.rightTrenchOpenStart),
          new Translation2d(
              FieldConstants.LinesVertical.neutralZoneNear + bufferTrenchX,
              FieldConstants.LinesHorizontal.rightTrenchOpenEnd),
        };

    Translation2d[] leftTrenchEdgesRED = new Translation2d[leftTrenchEdgesBLUE.length];
    Translation2d[] rightTrenchEdgesRED = new Translation2d[rightTrenchEdgesBLUE.length];

    for (int i = 0; i < leftTrenchEdgesBLUE.length; i++) {
      leftTrenchEdgesRED[i] = FlippingUtil.flipFieldPosition(leftTrenchEdgesBLUE[i]);
    }
    for (int i = 0; i < rightTrenchEdgesBLUE.length; i++) {
      rightTrenchEdgesRED[i] = FlippingUtil.flipFieldPosition(rightTrenchEdgesBLUE[i]);
    }

    this.transformedLeftTrenchZoneBLUE = new Region2d(leftTrenchEdgesBLUE);
    this.transformedRightTrenchZoneBLUE = new Region2d(rightTrenchEdgesBLUE);
    this.transformedLeftTrenchZoneRED = new Region2d(leftTrenchEdgesRED);
    this.transformedRightTrenchZoneRED = new Region2d(rightTrenchEdgesRED);
  }

  public void populateBumpZone() {

    double bufferBumpX = Units.inchesToMeters(BUMP_ZONE_BUFFER_X_INCHES);

    Translation2d[] leftBumpEdges =
        new Translation2d[] {

          // Left Bump
          new Translation2d(
              FieldConstants.LinesVertical.neutralZoneNear + bufferBumpX,
              FieldConstants.LinesHorizontal.leftBumpEnd),
          new Translation2d(
              FieldConstants.LinesVertical.neutralZoneNear + bufferBumpX,
              FieldConstants.LinesHorizontal.leftBumpStart),
          new Translation2d(
              FieldConstants.LinesVertical.allianceZone - bufferBumpX,
              FieldConstants.LinesHorizontal.leftBumpStart),
          new Translation2d(
              FieldConstants.LinesVertical.allianceZone - bufferBumpX,
              FieldConstants.LinesHorizontal.leftBumpEnd)
        };

    Translation2d[] rightBumpEdges =
        new Translation2d[] {
          // Right Bump
          new Translation2d(
              FieldConstants.LinesVertical.neutralZoneNear + bufferBumpX,
              FieldConstants.LinesHorizontal.rightBumpStart),
          new Translation2d(
              FieldConstants.LinesVertical.neutralZoneNear + bufferBumpX,
              FieldConstants.LinesHorizontal.rightBumpEnd),
          new Translation2d(
              FieldConstants.LinesVertical.allianceZone - bufferBumpX,
              FieldConstants.LinesHorizontal.rightBumpEnd),
          new Translation2d(
              FieldConstants.LinesVertical.allianceZone - bufferBumpX,
              FieldConstants.LinesHorizontal.rightBumpStart)
        };

    Translation2d[] leftBumpEdgesRED = new Translation2d[leftBumpEdges.length];
    Translation2d[] rightBumpEdgesRED = new Translation2d[rightBumpEdges.length];

    for (int i = 0; i < leftBumpEdges.length; i++) {
      leftBumpEdgesRED[i] = FlippingUtil.flipFieldPosition(leftBumpEdges[i]);
    }
    for (int i = 0; i < rightBumpEdges.length; i++) {
      rightBumpEdgesRED[i] = FlippingUtil.flipFieldPosition(rightBumpEdges[i]);
    }

    this.transformedLeftBumpZoneBLUE = new Region2d(leftBumpEdges);
    this.transformedRightBumpZoneBLUE = new Region2d(rightBumpEdges);
    this.transformedLeftBumpZoneRED = new Region2d(leftBumpEdgesRED);
    this.transformedRightBumpZoneRED = new Region2d(rightBumpEdgesRED);
  }

  public void logAllianceZonePoints() {
    transformedAllianceZone.logPoints("aliianceZone");
  }

  public void logTrenchZonePoints() {
    transformedLeftTrenchZoneBLUE.logPoints("leftTrenchZone BLUE");
    transformedRightTrenchZoneBLUE.logPoints("rightTrenchZone BLUE");
    transformedLeftTrenchZoneRED.logPoints("leftTrenchZone RED");
    transformedRightTrenchZoneRED.logPoints("rightTrenchZone RED");
  }

  public void logBumpZonePoints() {
    transformedLeftBumpZoneBLUE.logPoints("leftBumpZone BLUE");
    transformedRightBumpZoneBLUE.logPoints("rightBumpZone BLUE");
    transformedLeftBumpZoneRED.logPoints("leftBumpZone RED");
    transformedRightBumpZoneRED.logPoints("rightBumpZone RED");
  }

  /**
   * Create a path from a starting pose in one region to an ending pose in another region that
   * passes through the transition points defined for those regions.
   *
   * @param start the starting pose
   * @param end the ending pose
   * @param pathConstants the path constraints (i.e., max velocity, max acceleration)
   * @param subsystem the drivetrain subsystem
   * @return the path from the starting pose to the ending pose; null if no path exists
   */
  public PathPlannerPath makePath(
      Pose2d start, Pose2d end, PathConstraints pathConstants, SwerveDrivetrain subsystem) {
    Region2d startRegion = null;
    Region2d endRegion = null;

    // find the starting and ending regions
    for (Region2d region : regions) {
      if (region.contains(start)) {
        startRegion = region;
      }
      if (region.contains(end)) {
        endRegion = region;
      }
    }

    // make sure both start and end are on the field
    if (startRegion == null || endRegion == null) return null;

    // BFS to find the shortest path to the end
    List<Region2d> path = breadthFirstSearch(startRegion, endRegion);
    if (path.isEmpty()) return null;

    // create point locations
    ArrayList<Translation2d> pointLocations = new ArrayList<>();

    // add the starting point
    pointLocations.add(start.getTranslation());

    // add all the transition points
    for (int i = 0; i < path.size() - 1; i++) {
      Region2d from = path.get(i);
      Region2d to = path.get(i + 1);
      pointLocations.add(from.getTransitionPoint(to));
    }

    // add a transition point if starting region & ending region same
    if (startRegion == endRegion) {
      pointLocations.add(
          new Translation2d((start.getX() + end.getX()) / 2, (start.getY() + end.getY()) / 2));
    }

    // add the ending point
    pointLocations.add(end.getTranslation());

    List<Pose2d> pathPoses = createPathPoses(pointLocations);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathPoses);
    return new PathPlannerPath(
        waypoints,
        pathConstants,
        null,
        new GoalEndState(
            RobotConfig.getInstance().getMoveToPathFinalVelocity(), end.getRotation()));
  }

  /**
   * Create the path points based on the starting and ending poses and the point locations. The path
   * will be created such that the first path point matches the robot's current heading and velocity
   * to ensure a smooth transition to the path. The the starting and ending poses have different
   * rotations, the change in rotation will occur between the first and second points. The final
   * speed of the robot will be as specified by the robot's configuration class'
   * getMoveToPathFinalVelocity method.
   *
   * @param start the starting pose
   * @param end the ending pose
   * @param subsystem the drivetrain subsystem
   * @param pointLocations the locations of the points in the path
   * @return the path points
   */
  private List<Pose2d> createPathPoses(ArrayList<Translation2d> pointLocations) {
    List<Pose2d> pathPoses = new ArrayList<>();
    Rotation2d lastHeading = null;
    for (int i = 0; i < pointLocations.size() - 1; i++) {
      double deltaX = pointLocations.get(i + 1).getX() - pointLocations.get(i).getX();
      double deltaY = pointLocations.get(i + 1).getY() - pointLocations.get(i).getY();
      lastHeading = new Rotation2d(deltaX, deltaY);
      pathPoses.add(
          new Pose2d(pointLocations.get(i).getX(), pointLocations.get(i).getY(), lastHeading));
    }

    // the final path point will match the ending pose's rotation and the velocity as specified by
    // the robot's configuration class' getMoveToPathFinalVelocity method.
    pathPoses.add(
        new Pose2d(
            pointLocations.get(pointLocations.size() - 1).getX(),
            pointLocations.get(pointLocations.size() - 1).getY(),
            lastHeading));

    return pathPoses;
  }

  private List<Region2d> breadthFirstSearch(Region2d start, Region2d end) {
    Queue<ArrayList<Region2d>> todo = new LinkedList<>();
    Set<Region2d> explored = new HashSet<>();

    // add the starting region to the set of explored regions
    explored.add(start);

    // if the path starts and ends in the same region, return that region
    if (start == end) {
      return new ArrayList<>(Arrays.asList(start));
    }

    todo.add(
        new ArrayList<>(Arrays.asList(start))); // add a path starting with startRegion to the list

    while (!todo.isEmpty()) { // while the list isn't empty, keep looking over the list.
      ArrayList<Region2d> path = todo.poll();
      Region2d region = path.get(path.size() - 1); // last region in the path

      for (Region2d other : region.getNeighbors()) {
        if (!explored.contains(other)) {
          ArrayList<Region2d> newPath = new ArrayList<>(path);
          newPath.add(other);

          if (other == end) {
            return newPath;
          }

          explored.add(other);
          todo.add(newPath);
        }
      }
    }
    return new ArrayList<>();
  }

  /**
   * This method should be invoked once the alliance color is known. Refer to the RobotContainer's
   * checkAllianceColor method for best practices on when to check the alliance's color. The
   * alliance color is needed when running auto paths as those paths are always defined for
   * blue-alliance robots and need to be flipped for red-alliance robots.
   *
   * @param newAlliance the new alliance color
   */
  public void updateAlliance(DriverStation.Alliance newAlliance) {
    this.alliance = newAlliance;
  }

  /**
   * Get the alliance color.
   *
   * @return the alliance color
   */
  public Alliance getAlliance() {
    return alliance;
  }

  public boolean inAllianceZone() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();

    if (getAlliance() == Alliance.Red) {
      pose = FlippingUtil.flipFieldPose(pose);
    }

    return transformedAllianceZone.contains(pose);
  }

  public boolean inTrenchZone() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();

    return transformedLeftTrenchZoneBLUE.contains(pose)
        || transformedRightTrenchZoneBLUE.contains(pose)
        || transformedLeftTrenchZoneRED.contains(pose)
        || transformedRightTrenchZoneRED.contains(pose);
  }

  public boolean inBumpZone() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();

    if (getAlliance() == Alliance.Red) {
      pose = FlippingUtil.flipFieldPose(pose);
    }

    return transformedLeftBumpZoneBLUE.contains(pose)
        || transformedRightBumpZoneBLUE.contains(pose)
        || transformedLeftBumpZoneRED.contains(pose)
        || transformedRightBumpZoneRED.contains(pose);
  }

  public Pose2d getNearestBank() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();

    return pose.nearest(Arrays.asList(banks));
  }

  public enum Side {
    LEFT,
    RIGHT
  }
}
