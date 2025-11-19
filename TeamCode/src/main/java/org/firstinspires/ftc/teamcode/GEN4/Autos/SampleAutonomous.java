package org.firstinspires.ftc.teamcode.GEN4.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GEN4.Subsystems.Robot;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Sample Autonomous", group = "GEN4")
public class SampleAutonomous extends LinearOpMode {

    Robot robot;

    // TODO: MAKE SURE THESE ARE THE EXACT SAME LIST!!!!! OR ELSE GOONER
    public enum State {
        INITIALIZED,
        START_POSE,
        SHOOT_FIRST_THREE,
        INTAKE_STACK_1_a,
        INTAKE_STACK_1_b,
        SHOOT_POSE_a,
        SHOOT_POSE_b
    }
    public State state = State.START_POSE;

    // removed the list of strings — enum is the source of truth now



    // TODO: =============== SAMPLES ===============
    /*
        robot.goToPoint(new Pose2D(DistanceUnit.INCH, -20, -60, AngleUnit.DEGREES, -90), 0.8, 10, 15);
        if (atPosition) { state = State.STATEEE; }
     */
    // TODO: =======================================


    // nextState() helper to move through the enum list in order
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
                robot.drivetrain.setPosition(0,0,0); // START POSE
                // DETECT RANDOMIZATION
                // MOVE GANTRY
                /*
                if tagid = 21 {
                    goto SLOT X
                    premove 1 = x
                    premove 2 = x
                    premove 3 = x

                    sequence =
                    333 (insert real sequence)
                    233
                    133
                }


                 */
                // TELEMETRY ADD: balls in GPP formation, apriltag detected, pinpoint reset
                break;

            case START_POSE:

                nextState();
                break;

            case SHOOT_FIRST_THREE:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -20, 15, AngleUnit.DEGREES, 30), 0.9, 8, 10);
                if (atPosition) { nextState(); } break;

            case INTAKE_STACK_1_a:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -20, -60, AngleUnit.DEGREES, 90), 0.8, 10, 8);

                if (atPosition) { nextState(); } break;

            case INTAKE_STACK_1_b:
                robot.goToPoint(new Pose2D(DistanceUnit.INCH, -50, 12, AngleUnit.DEGREES, -90), 0.4, 2, 15);
                robot.intake.runIntake(1);

                if (atPosition) { nextState(); } break;

            // keep your structure intact
            case SHOOT_POSE_a:
                // your code here
                break;

            case SHOOT_POSE_b:
                // your code here
                break;
        }
    }

    @Override
    public void runOpMode() {

        //gotta do init
        robot = new Robot(hardwareMap);
        robot.startup();

        while (opModeInInit()) {
            state = State.INITIALIZED;
            updateSequence();
        }
        waitForStart();

        // loop
        while (opModeIsActive()) {
            updateSequence();
            telemetry.addData("ROBOT POSE:", robot.drivetrain.robotPose);
            telemetry.update();
        }
    }
}
