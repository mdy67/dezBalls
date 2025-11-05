package org.firstinspires.ftc.teamcode.Main.autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CONFIG.RobotConfig;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Utils.PoseStorage;
import org.firstinspires.ftc.teamcode.Main.Utils.TagProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "9 CLOSE 3 GOON SOCK", group = "Autonomous")
public class UndeCLOSEtedAuto3goonSock extends LinearOpMode {

    private Robot robot;
    private MecanumDrive drive;
    private TagProcessor tagProcessor;
    private int nextGantry = 0;
    private int detectedTag = 21; // default to 21

    // -------------------------------
    // Reusable Action Wrappers
    // -------------------------------

    public static class RunnableAction implements Action {
        private final Runnable runnable;
        private boolean done = false;

        public RunnableAction(Runnable runnable) {
            this.runnable = runnable;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!done) {
                runnable.run();
                done = true;
            }
            return false;
        }
    }

    public static class UpdatingAction implements Action {
        private final Action inner;
        private final Robot robot;

        public UpdatingAction(Action inner, Robot robot) {
            this.inner = inner;
            this.robot = robot;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            robot.diffy.update();
            robot.update();
            robot.flywheel.update();
            return inner.run(packet);
        }
    }

    public static class TimedFlickAction implements Action {
        private final Runnable flick;
        private final Runnable reset;
        private final double duration;
        private double startTime = 0;
        private boolean started = false;

        public TimedFlickAction(Runnable flick, Runnable reset, double duration) {
            this.flick = flick;
            this.reset = reset;
            this.duration = duration;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            double now = System.nanoTime() / 1e9;
            if (!started) {
                flick.run();
                startTime = now;
                started = true;
            }
            if (now - startTime > duration) {
                reset.run();
                return false;
            }
            return true;
        }
    }

    // -------------------------------
    // Action Factory Methods
    // -------------------------------

    private RunnableAction runIndexer(double power) { return new RunnableAction(() -> robot.intake.runIndexerOnly(power)); }
    private RunnableAction runIntake(double power) { return new RunnableAction(() -> robot.intake.runIntakeOnly(power)); }
    private RunnableAction spinFlywheel(double radPerSec) { return new RunnableAction(() -> robot.flywheel.setTargetVelocity(radPerSec)); }
    private RunnableAction turretAngle(double angleDeg) { return new RunnableAction(() -> robot.diffy.setAngle(angleDeg)); }
    private RunnableAction gantrySlot(int slot) { return new RunnableAction(() -> robot.diffy.goToSlot(slot)); }
    private RunnableAction spatulaOn() { return new RunnableAction(() -> robot.spatula.spatulaON()); }
    private RunnableAction spatulaOff() { return new RunnableAction(() -> robot.spatula.spatulaOFF()); }
    private RunnableAction resetArms() { return new RunnableAction(() -> robot.arms.reset()); }

    // -------------------------------
    // NEXT GANTRY FEATURE
    // -------------------------------
    private RunnableAction setNextGantry(int slot) { return new RunnableAction(() -> this.nextGantry = slot); }
    private RunnableAction goToNextGantrySlot() { return new RunnableAction(() -> robot.diffy.goToSlot(nextGantry)); }

    private TimedFlickAction flickArm3() { return new TimedFlickAction(() -> robot.arms.flickArm3(), () -> robot.arms.reset(), 0.35); }
    private TimedFlickAction flickArm2() { return new TimedFlickAction(() -> robot.arms.flickArm2(), () -> robot.arms.reset(), 0.35); }
    private TimedFlickAction flickArm1() { return new TimedFlickAction(() -> robot.arms.flickArm1(), () -> robot.arms.reset(), 0.35); }

    // -------------------------------
    // MAIN AUTO
    // -------------------------------

    @Override
    public void runOpMode() {
        Pose2d startPose;

        robot = new Robot(hardwareMap);
        tagProcessor = new TagProcessor(hardwareMap);
        robot.arms.reset();
        robot.diffy.reset(0);
        robot.spatula.spatulaOFF();

        // --- INIT LOOP ---
        while (opModeInInit()) {
            tagProcessor.handleInitLoop(this);
        }

        // --- USE LAST DETECTED TAG ---
        int tagFromCamera = tagProcessor.getDetectedTagId();
        if (tagFromCamera != -1) detectedTag = tagFromCamera;

        TagProcessor.Alliance alliance = tagProcessor.getAlliance();

        RobotConfig.alliance = (alliance == TagProcessor.Alliance.BLUE)
                ? RobotConfig.Alliance.BLUE
                : RobotConfig.Alliance.RED;

        if (RobotConfig.alliance == RobotConfig.Alliance.BLUE)
            startPose = new Pose2d(-16.7, -62.5, Math.toRadians(90));
        else
            startPose = new Pose2d(16.7, -62.5, Math.toRadians(90));

        drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("=== FINAL INIT DATA ===");
        telemetry.addData("Alliance", RobotConfig.alliance);
        telemetry.addData("Detected Tag", (detectedTag == -1) ? "None" : detectedTag);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ===============================
        // BLUE SEQUENCES
        // ===============================


        // TODO: ============= FIRST STACK INTAKE =======================


        // TODO: ========================= SECOND STACK INTAKE ===============================





        // TODO: ========================= THIRD STACK INTAKE ===============================
        // TODO: =================================== MOVE OFF LINE =============================



        // ===============================
        // SHOOTING SEQUENCES
        // ===============================
        Action shoot_133 = new UpdatingAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .afterTime(0.0, spatulaOff())
                        .afterTime(0.0, gantrySlot(1))
                        .afterTime(0.0, runIndexer(-0.7))
                        .afterTime(0.8, flickArm1())
                        .afterTime(1.25, gantrySlot(3))
                        .afterTime(1.25, spatulaOn())
                        .afterTime(1.25, runIndexer(-1.0))
                        .afterTime(2, flickArm3())
                        .afterTime(3, flickArm3())
                        .afterTime(0, setNextGantry(3)) // SET NEXT GANTRY ACCORDING TO MATRIX
                        .waitSeconds(3.4)
                        .build(),
                robot
        );

        Action shoot_223 = new UpdatingAction( // Same as PPG Preload
                drive.actionBuilder(drive.localizer.getPose())
                        .afterTime(0.0, spatulaOff())
                        .afterTime(0.0, gantrySlot(2))
                        .afterTime(0.0, runIndexer(-0.5))
                        .afterTime(0.8, flickArm2())
                        .afterTime(1, runIndexer(-1.0))
                        .afterTime(2, flickArm2())
                        .afterTime(2.4, gantrySlot(3))
                        .afterTime(2.4, spatulaOn())
                        .afterTime(3.25, flickArm3())
                        .afterTime(0, setNextGantry(1)) // SET NEXT GANTRY ACCORDING TO MATRIX
                        .waitSeconds(3.7)
                        .build(),
                robot
        );

        Action shoot_233 = new UpdatingAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .afterTime(0.0, spatulaOff())
                        .afterTime(0.0, gantrySlot(2))
                        .afterTime(0.0, runIndexer(-0.7))
                        .afterTime(0.8, flickArm2())
                        .afterTime(1.25, gantrySlot(3)) // YOUR PHONE LINGING BIG BOY
                        .afterTime(1.25, spatulaOn())
                        .afterTime(1.25, runIndexer(-1.0))
                        .afterTime(1.75, flickArm3()) //
                        .afterTime(2.1, runIndexer(-0.7))
                        .afterTime(2.5, runIndexer(-1.0))
                        .afterTime(3, flickArm3())
                        .afterTime(0, setNextGantry(3)) // SET NEXT GANTRY ACCORDING TO MATRIX
                        .waitSeconds(3.3)
                        .build(),
                robot
        );

        Action shoot_333 = new UpdatingAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(3))
                        .afterTime(0.0, runIndexer(-0.7))
                        .afterTime(1, flickArm3())
                        .afterTime(2.25, flickArm3())
                        .afterTime(2.5, runIndexer(-1.0))
                        .afterTime(3.5, flickArm3())
                        .afterTime(0, setNextGantry(2))
                        .waitSeconds(4)
                        .build(),
                robot
        );

        Action bluePreloadMove = new UpdatingAction(
                drive.actionBuilder(startPose)
                        .afterTime(0.0, spinFlywheel(340))
                        .afterTime(0.0, gantrySlot(tagProcessor.goToSlot))
                        .afterTime(0.0, turretAngle(90))
                        .strafeToSplineHeading(new Vector2d(-16, 0), Math.toRadians(130),
                                new TranslationalVelConstraint(80),
                                new ProfileAccelConstraint(-80, 80))
                        .strafeToLinearHeading(new Vector2d(-16, 12), Math.toRadians(130),
                                new TranslationalVelConstraint(30),
                                new ProfileAccelConstraint(-30, 30))
                        .build(),
                robot
        );

        Action blueSecondStackIntake = new UpdatingAction(
                drive.actionBuilder(new Pose2d(-16, 12, Math.toRadians(130)))
                        .afterTime(0.0, spatulaOff())
                        .afterTime(0.0, gantrySlot(tagProcessor.goToSlot))
                        .afterTime(0.0, spinFlywheel(120))
                        .afterTime(0.0, flickArm3())
                        .afterTime(0, runIntake(1.0))
                        .afterTime(0, runIndexer(-1.0))
                        .afterTime(0.25, flickArm2())
                        .afterTime(0.5, flickArm1())
                        .afterTime(2, turretAngle(90))
                        .afterTime(3, spinFlywheel(340))

                        .strafeToLinearHeading(new Vector2d(-20, 12), Math.toRadians(180),
                                new TranslationalVelConstraint(60),
                                new ProfileAccelConstraint(-60, 60))

                        .strafeToLinearHeading(new Vector2d(-52, 12), Math.toRadians(180),
                                new TranslationalVelConstraint(20),
                                new ProfileAccelConstraint(-20, 20))
                        .strafeToLinearHeading(new Vector2d(-16, 0), Math.toRadians(130),
                                new TranslationalVelConstraint(60),
                                new ProfileAccelConstraint(-60, 60))
                        .strafeToLinearHeading(new Vector2d(-16, 12), Math.toRadians(130),
                                new TranslationalVelConstraint(30),
                                new ProfileAccelConstraint(-30, 30))

                        .build(),
                robot
        );

        Action blue_thirdStackIntake = new UpdatingAction(
                drive.actionBuilder(new Pose2d(-16, 12, Math.toRadians(130)))
                        .afterTime(0.0, spatulaOff())
                        .afterTime(0.0, gantrySlot(tagProcessor.goToSlot))
                        .afterTime(0.0, spinFlywheel(120))
                        .afterTime(0.0, flickArm3())
                        .afterTime(0.25, flickArm2())
                        .afterTime(0.5, flickArm1())
                        .afterTime(2, turretAngle(90))
                        .afterTime(0, runIntake(1.0))
                        .afterTime(0, runIndexer(-1.0))
                        .afterTime(3, spinFlywheel(340))
                        .strafeToLinearHeading(new Vector2d(-16, -12), Math.toRadians(180),
                                new TranslationalVelConstraint(60),
                                new ProfileAccelConstraint(-60, 60))
                        .strafeToLinearHeading(new Vector2d(-59, -12), Math.toRadians(180), // WALL [INTAKED] [AGAINST GOAL BC TOP STACK]
                                new TranslationalVelConstraint(40),
                                new ProfileAccelConstraint(-40, 40))
                        .strafeToLinearHeading(new Vector2d(-16, 0), Math.toRadians(130),
                                new TranslationalVelConstraint(60),
                                new ProfileAccelConstraint(-60, 60))
                        .strafeToLinearHeading(new Vector2d(-16, 12), Math.toRadians(130),
                                new TranslationalVelConstraint(30),
                                new ProfileAccelConstraint(-30, 30))
                        .build(),
                robot
        );



        // ===============================
        // far auto
        // ===============================
        if (RobotConfig.alliance == RobotConfig.Alliance.BLUE) {

            if (detectedTag == 21) tagProcessor.goToSlot = 3; // GPP
            else if (detectedTag == 22) tagProcessor.goToSlot = 2; // PGP
            else if (detectedTag == 23) tagProcessor.goToSlot = 2; // PPG
            else tagProcessor.goToSlot = 3;

            Actions.runBlocking(bluePreloadMove);

            if (detectedTag == 21) Actions.runBlocking(shoot_333); // GPP
            else if (detectedTag == 22) Actions.runBlocking(shoot_233); // PGP
            else if (detectedTag == 23) Actions.runBlocking(shoot_223); // PPG
            else Actions.runBlocking(shoot_333);

            if (detectedTag == 21) tagProcessor.goToSlot = 1; // GPP [133]
            else if (detectedTag == 22) tagProcessor.goToSlot = 2; // PGP [223]
            else if (detectedTag == 23) tagProcessor.goToSlot = 3; // PPG [333]
            else tagProcessor.goToSlot = 3;

            Actions.runBlocking(blueSecondStackIntake);

            if (detectedTag == 21) Actions.runBlocking(shoot_133); // GPP
            else if (detectedTag == 22) Actions.runBlocking(shoot_223); // PGP
            else if (detectedTag == 23) Actions.runBlocking(shoot_333); // PPG
            else Actions.runBlocking(shoot_333);

            if (detectedTag == 21) tagProcessor.goToSlot = 2; // GPP [233]
            else if (detectedTag == 22) tagProcessor.goToSlot = 3; // PGP [333]
            else if (detectedTag == 23) tagProcessor.goToSlot = 1; // PPG [133]
            else tagProcessor.goToSlot = 3;

            Actions.runBlocking(blue_thirdStackIntake);

            if (detectedTag == 21) Actions.runBlocking(shoot_233); // GPP
            else if (detectedTag == 22) Actions.runBlocking(shoot_333); // PGP
            else if (detectedTag == 23) Actions.runBlocking(shoot_133); // PPG
            else Actions.runBlocking(shoot_333);

        } else if (RobotConfig.alliance == RobotConfig.Alliance.RED) {




        }

        // Final updates
        PoseStorage.storedPose = drive.localizer.getPose();

        robot.end();
        robot.update();
        robot.flywheel.update();
        robot.diffy.update();
        tagProcessor.close();

    }
}
