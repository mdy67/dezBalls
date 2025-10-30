package org.firstinspires.ftc.teamcode.Main.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Diffy;

@Config
@TeleOp(name = "TELEOP", group = "Main")
public class TELEOP extends com.qualcomm.robotcore.eventloop.opmode.OpMode {

    private Robot robot;

    // Turret limits
    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 180;

    // Flywheel tuning (dashboard adjustable)
    public static double baseVelocity = 200;
    public static double distanceCoefficient = 1.3;
    public static double maxAllowedVelocity = 580;

    // Increment rates
    public static double baseVelocityIncrement = 1;
    public static double distanceCoefficientIncrement = 0.1;

    private double angleOffset = 0;
    private boolean angleOffsetTrigger = false;
    public boolean fixedTurretTrigger = true;
    private static final double FIXED_SHOOTER_VELOCITY = 350;
    public boolean dontTouchLittleChildren = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.arms.reset();
        robot.diffy.reset(0);

        telemetry.addLine("TELEOP Initialized - Base & Coefficient Adjustable via Gamepad2");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (dontTouchLittleChildren) {
            robot.update();

            // --- DRIVE CONTROL ---
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            boolean slowMode = gamepad1.right_trigger > 0.1;
            robot.drivetrain.move(drive, strafe, rotate, slowMode);

            // --- INTAKE ---
            robot.intake.teleUpdate(gamepad1.left_bumper, gamepad1.dpad_down);

            // --- ARM FLICK & INDEXER ---
            if (gamepad1.a) {
                robot.intake.runIndexerOnly(-0.4);
                robot.arms.flickArm3();
                robot.intake.runIndexerOnly(robot.intake.getIndexerSpeed());
            }

            if (gamepad1.dpad_right) {
                fixedTurretTrigger = true;
            } if (gamepad1.dpad_left) {
                fixedTurretTrigger = false;
            }

            // --- DISTANCE & TURRET CONTROL ---
            double goalX = Diffy.BLUE_GOAL_X;
            double goalY = Diffy.BLUE_GOAL_Y;

            if (!gamepad1.x && !gamepad1.b) {
                angleOffsetTrigger = false;
            }

            if (gamepad1.x) {
                if (!angleOffsetTrigger) {
                    angleOffsetTrigger = true;
                    angleOffset -= 1;
                }

            } else if (gamepad1.b) {
                if (!angleOffsetTrigger) {
                    angleOffsetTrigger = true;
                    angleOffset += 1;
                }

            }



            double dx = goalX - robot.xPOS;
            double dy = goalY - robot.yPOS;
            double distanceInches = Math.sqrt(dx*dx + dy*dy);

            double turretAngle = robot.diffy.aimTurretAngle(robot.xPOS, robot.yPOS, robot.headingRad);
            turretAngle = Range.clip(turretAngle, MIN_ANGLE, MAX_ANGLE);
            if (!fixedTurretTrigger) {
                robot.diffy.setAngle(turretAngle + angleOffset);
            } else {
                robot.diffy.setAngle(90);
            }

            robot.diffy.update();

            // --- ADJUST BASE VELOCITY & DISTANCE COEFFICIENT ---
            baseVelocity += -gamepad2.right_stick_y * baseVelocityIncrement;
            distanceCoefficient += -gamepad2.left_stick_y * distanceCoefficientIncrement;

            baseVelocity = Range.clip(baseVelocity, 0, maxAllowedVelocity);
            distanceCoefficient = Math.max(distanceCoefficient, 0);

            // --- FLYWHEEL CONTROL ---


            double a = 0.0000413348;
            double b = -0.00206321;
            double c = 0.548967;
            double d = 299.02768;

            // ax^3 + bx^2 + cx + d
            double regressionVelocity = ((a * (Math.pow(distanceInches, 3))) + (b * Math.pow(distanceInches, 2)) + ((c * distanceInches) + d));

            if (!fixedTurretTrigger) {
                robot.flywheel.setTargetVelocity(regressionVelocity);
            } else {
                robot.flywheel.setTargetVelocity(FIXED_SHOOTER_VELOCITY);
            }

            robot.flywheel.update();

            // --- TELEMETRY ---
            telemetry.addData("Distance to Goal", "%.2f in", distanceInches);
            telemetry.addData("Turret Angle", "%.2f", turretAngle + angleOffset);
            telemetry.addData("Regression Velocity", regressionVelocity);
            telemetry.addData("Flywheel Actual", "%.2f rad/s", robot.flywheel.getVelocityRadPerSec());
            telemetry.addLine();
            telemetry.addLine("======== IMPORTANT INFORMATION ========");
            telemetry.addData("Fixed Turret T/F:", fixedTurretTrigger);
            telemetry.addData("ROBOT X:", robot.xPOS);
            telemetry.addData("ROBOT Y:", robot.yPOS);
            telemetry.addData("ROBOT HEADING:", Math.toDegrees(robot.headingRad));
            telemetry.update();
        }
        if (Math.abs(gamepad1.right_stick_x * gamepad1.right_stick_y * gamepad1.left_stick_x * gamepad1.left_stick_y * 100000000000000000f) > 0.001) {
            dontTouchLittleChildren = true; // trigger for movement
        }

    }
}
