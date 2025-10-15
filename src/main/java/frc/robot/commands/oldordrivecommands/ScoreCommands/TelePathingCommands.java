package frc.robot.commands.oldordrivecommands.ScoreCommands;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.ArrayList;
import java.util.Comparator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.TelePathingConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TelePathingCommands {

    SwerveSubsystem swerve;

    public TelePathingCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }



    private Supplier<Command> autoDriveCommandSupplier = () -> new InstantCommand();
    private int autoDriveReefSubPos = 0;
    private int autoDriveCoralSide = 0;

    /**
     * The path-follow command is generated every time this command initializes, not when it is first created.
     * Requires the swerve subsystem.
     */
    public Command getAutoDriveDeferredCommand() {
        // this::getAutoDriveCommand is used instead of autoDriveCommandSupplier because the value of autoDriveCommandSupplier may change after this runs.
        return new DeferredCommand(this::getAutoDriveCommand, Set.of(swerve));
    }

    public Command getAutoDriveCommand() {
        return autoDriveCommandSupplier.get();
    }
    
    public void setAutoDriveGoToReef(int index) {
        autoDriveCommandSupplier = () -> goToReefSmartRelativeCommand(index, autoDriveReefSubPos);
    }

    public void setAutoDriveGoToCoralStation() {
        autoDriveCommandSupplier = () -> goToCoralStationSmartRelativeCommand(autoDriveCoralSide);
    }

    public void setAutoDriveGoToBarge() {
        autoDriveCommandSupplier = () -> goToBargeSmartCommand();
    }

    public void setAutoDriveNone() {
        autoDriveCommandSupplier = () -> new InstantCommand();
    }

    public void setAutoDriveSide(int side) {
        autoDriveReefSubPos = side;
        if (side == 1) autoDriveCoralSide = 1;
        else if (side == -1) autoDriveCoralSide = 0;
    }



    /**
     * index is which side of the reef the robot is going to, where 0 is the barge-side, 3 is the far-side,
     * 1/2 are on the left side, and 4/5 are on the right side.
     * subPos is where on the edge of the reef the robot will go, where 0 is the center,
     * -1 is to the left of the driver, and 1 is to the right of the driver.
     */
    private Command goToReefSmartRelativeCommand(int index, int subPos) {
        if (index == 0 || index == 1 || index == 5) subPos *= -1;
        //if (getAllianceSimple()) {
        //    return GoToReefSmartCommand((6 - index) % 6, -subPos);
        //} else {
            return GoToReefSmartCommand(index, subPos);
        //}
    }




    /**
     * 0 is left side, 1 is right side.
     */
    public Command goToCoralStationSmartRelativeCommand(int side) {
        //if (getAllianceSimple()) {
        //    return goToCoralStationSmartCommand(side);
        //} else {
            return goToCoralStationSmartCommand((side + 1) % 2);
        //}
    }






    /**
     * @return true if on red alliance (path will be flipped), false if on blue or invalid alliance
     */
    private static boolean getAllianceSimple() {
        // Why in the world does DriverStation.getAlliance() return an enum wrapped in an Optional???
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
        {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }




    public Command GoToReefCommand(int index) {

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        Pose2d dest = new Pose2d(
            1.75*Math.cos(index * Math.PI/3) + 4.5,
            1.75*Math.sin(index * Math.PI/3) + 4.0,
            Rotation2d.fromDegrees(180 + index*60));
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                getRobotPose(dest.getTranslation()),
                dest
        );

        //PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                TelePathingConstants.kDefaultConstraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(180 + index*60)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Turns the path into a command
        return AutoBuilder.followPath(path);

    }


    /**
     * It looks really bad and complex, because it is
     * but basically it takes the target point, the robot position, and determines
     * whether or not to go in a straight line (canGoStraight).
     * If it can't go straight, it will determine the fastest direction to go around the reef (goCounterCW)
     * After determining the direction, it loops through the corners of the reef to see which are
     * between the current angle from the reef and the target angle
     * then sorts them into a list by distance from the robot
     * then creates a list of poses (I'm not sure but the robot might end up backwards as it circles the reef)
     *
     * index is which side of the reef the robot is going to, where 0 is the barge-side, 3 is the far-side,
     * 1/2 are on the blue-side, and 4/5 are on the red side
     * subPos is where on the edge of the reef the robot will go, where 0 is the center,
     * 1 is the coral pole in the positive direction (index 1 to 2 to 3 etc)
     *
     * There's probably a mistake, but I did test it in IntelliJ and it seemed alright
     * There was also probably a simpler way but oh well
     */
    public Command GoToReefSmartCommand(int index, int subPos) {

        PathPlannerPath path;

        //finDist is the distance from the final point of the path to the center of the reef
        //waypntDist is the distance from the waypoints the robot uses to circle the reef from the center of the reef
        double finDist = 1.384;
        double waypntDist = 2.0;
        double adjustDist = 0.144;
     

        //The current position of the robot
        double roboX = swerve.getPose().getX();
        double roboY = swerve.getPose().getY();

        //The final position the robot will end at ((4.5, 4.0) is the center of the reef)
        double targX = finDist * Math.cos(index * Math.PI / 3) + 4.5;
        double targY = finDist * Math.sin(index * Math.PI / 3) + 4.0;

        //This calculates a line, parallel to the reef,
        //and centered at the targetpoint. Then it calculates the distance
        //of the x position of the robot from the x position of the line
        //at the y position of the robot (The x(y))
        double tangent = -Math.tan(index * Math.PI / 3);
        double pointDifference = roboX - (targX + tangent*(roboY-targY));

        double slope = index * Math.PI / 3;

        //If the index is 0, 1, or 5, the robot needs to be to the right
        //of the line created above to be able to get to the final point
        //in a straight line. 
        boolean canGoStraight;
        if (index <= 1 || index == 5) {
            canGoStraight = pointDifference > 0;
        } else {
            canGoStraight = pointDifference < 0;
        }


        //The code before the 'else' is basically the same as the simple path
        if (canGoStraight) {


            Pose2d dest = new Pose2d(
                finDist*Math.cos(index * Math.PI/3) + 4.5 - subPos*adjustDist*Math.sin(index * Math.PI/3),
                finDist*Math.sin(index * Math.PI/3) + 4.0 + subPos*adjustDist*Math.cos(index * Math.PI/3),
                Rotation2d.fromDegrees(180 + index*60));

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    getRobotPose(dest.getTranslation()),
                    dest
            );

            //PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

            // Create the path using the waypoints created above
            path = new PathPlannerPath(
                    waypoints,
                    TelePathingConstants.kDefaultConstraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                    new GoalEndState(0.0, Rotation2d.fromDegrees(180 + index*60)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );


        } else {

            //Creates the angle from the center of the reef to the robot
            //counterCW is counter clockwise
            double angle = Math.atan2(roboY - 4.0, roboX - 4.5);
            if (angle < 0) {
                angle += Math.PI*2;
            }
            boolean goCounterCW;

            //Determines which direction to go. Honestly I'm not 100% sure why it works
            if (angle > slope) {
                double counterCW = Math.abs(angle - (slope + 2 * Math.PI));
                double clockwise = Math.abs(angle - slope);

                goCounterCW = counterCW < clockwise;
            } else {
                double clockwise = Math.abs(angle - slope);
                double counterCW = Math.abs(angle - (slope - 2 * Math.PI));
                System.out.println(angle - (slope - 2 * Math.PI));

                goCounterCW = clockwise < counterCW;
            }

            ArrayList<Double> angleList = new ArrayList<>();


            //This is the really rough part, but it works.
            //Based on the angles and whether it can go clockwise, it chooses which code to run
            //Each one is fairly similar with slightly different parameters.
            //It loops through each possible index point, and determines whether its 'in the way'
            //by seeing if its between the angle of the robot and the angle of the target point.
            //If so, it adds it to angleList
            if (goCounterCW) {
                if (angle > slope) {
                    System.out.println(1);
                    for (int i = 0; i < 6; i++) {
                        if ((i * Math.PI / 3 + Math.PI/6) > angle || (i * Math.PI / 3 + Math.PI/6) < slope) {
                            angleList.add(i * Math.PI / 3 + Math.PI/6);
                        }
                    }
                } else {
                    System.out.println(2);
                    for (int i = 0; i < 6; i++) {
                        if ((i * Math.PI / 3 + Math.PI/6) > angle && (i * Math.PI / 3 + Math.PI/6) < slope) {
                            angleList.add(i * Math.PI / 3 + Math.PI/6);
                        }
                    }
                }
            } else {
                if (angle > slope) {
                    System.out.println(3);
                    for (int i = 0; i < 6; i++) {
                        if ((i * Math.PI / 3 + Math.PI/6) < angle && (i * Math.PI / 3 + Math.PI/6) > slope) {
                            angleList.add(i * Math.PI / 3 + Math.PI/6);
                        }
                    }
                } else {
                    System.out.println(4);
                    for (int i = 0; i < 6; i++) {
                        if ((i * Math.PI / 3 + Math.PI/6) < angle || (i * Math.PI / 3 + Math.PI/6) > slope) {
                            angleList.add(i * Math.PI / 3 + Math.PI/6);
                        }
                    }
                }
            }

            //This converts the angles gotten above into their distances from the robot
            //then it sorts them by value (smallest/closest first).
            //It does not actually change the values, as the final list is still a list of angles
            ArrayList<Double> sortedAngleList = angleList.stream()
                    .sorted(Comparator.comparing(ang -> Math.hypot(waypntDist * Math.cos(ang) + 4.5 - roboX, waypntDist * Math.sin(ang) + 4.0 - roboY)))
                    .collect(Collectors.toCollection(ArrayList::new));
            
            //Creates a list of poses from each angle from sortedAngleList
            //as well as adding the final point
            ArrayList<Pose2d> poseList = new ArrayList<>();
            if (goCounterCW) {
            for (Double ang : sortedAngleList) {
                poseList.add(new Pose2d(
                    waypntDist * Math.cos(ang) + 4.5,
                    waypntDist * Math.sin(ang) + 4.0,
                    Rotation2d.fromRadians(ang + Math.PI/2)
                ));
            }
            } else {
                for (Double ang : sortedAngleList) {
                    poseList.add(new Pose2d(
                        waypntDist * Math.cos(ang) + 4.5,
                        waypntDist * Math.sin(ang) + 4.0,
                        Rotation2d.fromRadians(ang - Math.PI/2)
                    ));
                }
            }
            poseList.add(new Pose2d(
                finDist*Math.cos(index * Math.PI/3) + 4.5 - subPos*adjustDist*Math.sin(index * Math.PI/3),
                finDist*Math.sin(index * Math.PI/3) + 4.0 + subPos*adjustDist*Math.cos(index * Math.PI/3),
                Rotation2d.fromDegrees(180 + index*60))
            );

            //This is always added, as it's the robot's starting pose
            poseList.add(0, getRobotPose(poseList.get(0).getTranslation()));

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poseList);

            //PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
    
            // Create the path using the waypoints created above
            path = new PathPlannerPath(
                    waypoints,
                    TelePathingConstants.kDefaultConstraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                    new GoalEndState(0.0, Rotation2d.fromDegrees(180 + index*60)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
        }
        

        return AutoBuilder.followPath(path);

    }














    
    
    /** Index 0 is the red-side coral station, 1 is the blue-side coral station */
    public Command goToCoralStationSmartCommand(int ind) {

        PathPlannerPath path;

        double roboX = swerve.getPose().getX();
        double roboY = swerve.getPose().getY();

        double targX = 1.223;
        double targY = 1.0 + 6*ind;

        boolean canGoStraight = false;

        //checks if the robot is below/above the reef
        if (ind == 0) {
            if (roboY < 3 || roboX < 3.666) {
                canGoStraight = true;
            }
        } else {
            if (roboY > 5 || roboX < 3.666) {
                canGoStraight = true;
            }
        }
        //checks if the robot is on the correct side of the reef
        if (roboX < 3.5) {
            canGoStraight = true;
        }


        ArrayList<Pose2d> poseList = new ArrayList<>();

        if (!canGoStraight) {
            //If its left of the the center of the reef, it will go around it on the non-barge-side
            if (roboX < 4.5) { 
                poseList.add(new Pose2d(
                    2.321,
                    4.795 - ind*(0.795*2),
                    Rotation2d.fromDegrees(-90 + 180*ind))
                );
            } else {
                //else, it will go around the reef barge-side
                //This if else determines if it needs to go closer to the barge before
                //traveling up/down past the reef
                if (ind == 0) {
                    if (roboY > 4.5) {
                        poseList.add(new Pose2d(
                            6.6,
                            4.6,
                            Rotation2d.fromDegrees(90))
                        );
                    }
                } else {
                    if (roboY < 3.5) {
                        poseList.add(new Pose2d(
                            6.6,
                            3.4,
                            Rotation2d.fromDegrees(-90))
                        );
                    }
                }
                //This goes up/down past the reef
                poseList.add(new Pose2d(
                    6.6,
                    1.5 + 5*ind,
                    Rotation2d.fromDegrees(180))
                );
            }

        }

        //This is always added, whether or not it can go straight, as its the coral station position
        poseList.add(new Pose2d(
            targX,
            targY,
            Rotation2d.fromDegrees(-125 + 250*ind))
        );

        //This is always added, as it's the robot's starting pose
        poseList.add(0, getRobotPose(poseList.get(0).getTranslation()));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poseList);

        //PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
    
        // Create the path using the waypoints created above
        path = new PathPlannerPath(
            waypoints,
            TelePathingConstants.kDefaultConstraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-125 + 250*ind)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );


        return AutoBuilder.followPath(path);

    }





    /**
     * Maintains the robot's Y position at time of call.
     * 
     * Drives to (flipped for alliance): X position 7.5, heading 0
     */
    public Command goToBargeSmartCommand() {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        Translation2d dest = new Translation2d(7.5, swerve.getPose().getY());
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            getRobotPose(dest),
            new Pose2d(dest, Rotation2d.fromDegrees(0))
        );

        //PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            TelePathingConstants.kDefaultConstraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        return AutoBuilder.followPath(path);
    }







    private Pose2d getRobotPose(Translation2d pointingTowards) {
        Translation2d robotPos = swerve.getPose().getTranslation();
        return new Pose2d(robotPos, pointingTowards.minus(robotPos).getAngle());
    }

}