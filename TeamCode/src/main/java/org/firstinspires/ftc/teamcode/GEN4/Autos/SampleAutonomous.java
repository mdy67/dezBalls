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
        POINT_3
    }

    private State state = State.START_POSE;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        robot.disableFlywheel(); // ENSURE FLYWHEEL NOT USED
        robot.startup();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        while (opModeInInit()) {
            state = State.INITIALIZED;
            robot.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            updateSequence();

            telemetry.addData("ROBOT X:", robot.drivetrain.robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("ROBOT Y:", robot.drivetrain.robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("ROBOT HEADING:", robot.drivetrain.robotPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("AUTO FSM: ", state);
            telemetry.addData("DRIVETRAIN FSM:", robot.drivetrain.state);
            telemetry.update();
        }
    }

    private void nextState() {
        int next = state.ordinal() + 1;
        if (next < State.values().length) {
            state = State.values()[next];
        }
    }

    private void updateSequence() {
        robot.update();
        boolean atPosition = robot.drivetrain.DTatTarget();

        switch (state) {
            case INITIALIZED:
                robot.drivetrain.setPosition(0,0,0);
                nextState();
                break;

            case START_POSE:
                nextState();
                break;

            case POINT_1:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, 20, 0, AngleUnit.DEGREES, 0), 0.6, 2, 5);
                if (atPosition) nextState();
                break;

            case POINT_2:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0), 0.6, 2, 5);
                if (atPosition) nextState();
                break;

            case POINT_3:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -20, 0, AngleUnit.DEGREES, 0), 0.6, 2, 5);
                if (atPosition) nextState();
                break;
        }
    }
}
