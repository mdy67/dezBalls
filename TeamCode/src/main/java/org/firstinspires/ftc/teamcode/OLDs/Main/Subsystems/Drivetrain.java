package org.firstinspires.ftc.teamcode.OLDs.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    DcMotorEx leftFront, leftBack, rightFront, rightBack;

    public Drivetrain(HardwareMap hardwareMap) {
        // CONFIG

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Moves the drivetrain using mecanum drive math, normalized for consistent ratios.
     *
     * @param drive    Forward/backward input (-gamepad1.left_stick_y)
     * @param strafe   Left/right input (gamepad1.left_stick_x)
     * @param rotate   Rotation input (gamepad1.right_stick_x)
     * @param slowMode True for slow mode, false for full speed
     */
    public void move(double drive, double strafe, double rotate, boolean slowMode) {
        double speedMultiplier = slowMode ? 0.3 : 1.0;

        double lf = drive + strafe + rotate * 1.5;
        double lb = drive - strafe + rotate * 1.5;
        double rf = drive - strafe - rotate * 1.5;
        double rb = drive + strafe - rotate * 1.5;

        // Normalize to prevent ratio distortion
        double max = Math.max(1.0, Math.abs(lf));
        max = Math.max(max, Math.abs(lb));
        max = Math.max(max, Math.abs(rf));
        max = Math.max(max, Math.abs(rb));

        lf /= max;
        lb /= max;
        rf /= max;
        rb /= max;

        leftFront.setPower(lf * speedMultiplier);
        leftBack.setPower(lb * speedMultiplier);
        rightFront.setPower(rf * speedMultiplier);
        rightBack.setPower(rb * speedMultiplier);
    }
}
