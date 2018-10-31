package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created 10-15-2018
 *
 * Test of lift arm using rack-n-pinion.  Uses two motors that are controlled
 * together using gamepad1 dpad up and down with power/speed control
 * via static variable armPower
 *
 * 10-30-2018:
 * Added touch sensor logic to stop lift when it reaches max height
 *     Touch sensor button pressed and sensor returns a false
 *
 * Version 1.2a
 */

@TeleOp

public class OnBotLiftArmTest extends LinearOpMode {

    private Gyroscope imu;
    public DcMotor  leftDrive;
    public DcMotor  rightDrive;
    public DcMotor  leftLift;
    public DcMotor  rightLift;
    public DigitalChannel digitalTouch;

    private Servo servoTest;

    @Override
    public void runOpMode () {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift = hardwareMap.get(DcMotor.class, "right_lift");

        // servoTest = hardwareMap.get(Servo.class, "servoTest");

        digitalTouch = hardwareMap.digitalChannel.get("touch_sensor");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized - All Systems Go!");
        telemetry.update();
        // Wait for game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double left = 0;
        double right = 0;
        int liftArm = 0;
        double powerOffset = 1;
        double directionSwap = 1;
        double liftPower = 0.20;
        boolean touchState = true;

        while (opModeIsActive()) {
            // Use gamepad button x to toggle a power offset to drive motors
            if (gamepad1.x) {
                if (powerOffset == 1) {
                    powerOffset = 0.5;
                } else {
                    powerOffset = 1;
                }
            }

            // Use gamepad button b to toggle direction of drive motors
            if (gamepad1.b) {
                sleep(150);
                if (directionSwap == 1) {
                    directionSwap = -1;
                } else {
                    directionSwap = 1;
                }
            }

            // Use gamepad button y to toggle power to lift motors
            if (gamepad1.y) {
                sleep(50);
                if (liftPower == 0.15) {
                    liftPower = 0.5;
                } else {
                    liftPower = 0.15;
                }
            }

            left = directionSwap * -this.gamepad1.left_stick_y;
            right = directionSwap * this.gamepad1.right_stick_y;
            leftDrive.setPower(left * powerOffset);
            rightDrive.setPower(right * powerOffset);

            // Control lift Arm
            touchState = digitalTouch.getState();

            if ( this.gamepad1.dpad_up && touchState == true ) {
                liftArm = 1;
                liftPower=0.30;
            } else if ( this.gamepad1.dpad_down ) {
                liftArm = -1;
                // liftPower=0.15;
            } else {
                liftArm = 0;
            }

            leftLift.setPower(liftArm * liftPower);
            rightLift.setPower(liftArm * -liftPower);

            telemetry.addData("Target Power - Left", left);
            telemetry.addData("Target Power - Right", right);
            telemetry.addData("Motor Power - Left", leftDrive.getPower());
            telemetry.addData("Motor Power - Right", rightDrive.getPower());
            telemetry.addData("Power Offset:", powerOffset);
            telemetry.addData("Lift Power", liftPower);
            telemetry.addData("Direction Swap", directionSwap );
            telemetry.addData("Touch Sensor", touchState );
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
