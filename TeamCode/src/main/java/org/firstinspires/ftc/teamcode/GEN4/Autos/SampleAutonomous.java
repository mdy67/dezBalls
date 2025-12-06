package org.firstinspires.ftc.teamcode.GEN4.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Drivetrain;

@Autonomous(name = "Sample Autonomous", group = "GEN4")
public class SampleAutonomous extends LinearOpMode {

    private Robot robot;

    public enum State {
        INITIALIZED,
        START_POSE,
        POINT_1,
        POINT_2,
        POINT_3,
        POINT_4,
        POINT_5,
        FINISHED
    }

    private State state = State.START_POSE;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        robot.disableFlywheel();
        robot.startup();

        while (opModeInInit()) {
            state = State.INITIALIZED;
            robot.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            updateSequence();

            // Telemetry for debugging
            telemetry.addData("AUTO FSM", state);
            telemetry.addData("DRIVETRAIN FSM", robot.drivetrain.state);
            telemetry.addData("ROBOT X", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("ROBOT Y", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("ROBOT HEADING", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("xPower", robot.drivetrain.xPower);
            telemetry.addData("yPower", robot.drivetrain.yPower);
            telemetry.addData("tPower", robot.drivetrain.tPower);
            telemetry.addData("WAIT ACTIVE", robot.wait.isActive());
            telemetry.addData("WAIT FINISHED", robot.wait.isFinished());
            telemetry.update();
        }
    }

    private void nextState() {
        int next = state.ordinal() + 1;
        if (next < State.values().length) {
            state = State.values()[next];
            robot.wait.reset(); // reset wait timer on new State
        }
    }

    private void updateSequence() {

        robot.update();
        boolean atPosition = robot.drivetrain.DTatTarget();

        switch (state) {

            case INITIALIZED:
                robot.drivetrain.setPosition(-17, -62, 270);
                nextState();
                break;

            case START_POSE:
                nextState();
                break;

            case POINT_1:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 12, AngleUnit.DEGREES, 240),
                        0.5, 3, 5);

                if (atPosition) {
                    if (!robot.wait.isActive()) {
                        robot.wait.waitSeconds(1); // 1 second wait
                    } else if (robot.wait.isFinished()) {
                        nextState();
                    }
                }
                break;

            case POINT_2:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 12, AngleUnit.DEGREES, 180),
                        0.25, 4, 5);
                if (atPosition) nextState();
                break;

            case POINT_3:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -40, 12, AngleUnit.DEGREES, 0),
                        0.25, 1, 5);
                if (atPosition) nextState();
                break;

            case POINT_4:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, 12, AngleUnit.DEGREES, 240),
                        0.4, 3, 5);
                if (atPosition) nextState();
                break;

            case POINT_5:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -18, -12, AngleUnit.DEGREES, 180),
                        0.4, 1, 5);
                if (atPosition) nextState();
                break;

            case FINISHED:
                robot.drivetrain.state = Drivetrain.State.IDLE;
                break;
        }
    }
}
