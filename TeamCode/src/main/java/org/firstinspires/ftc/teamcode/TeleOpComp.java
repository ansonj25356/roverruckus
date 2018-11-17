package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * 11-11-2018
 *
 * TeleOp for competition matches
 *   Gamepad1:  Drive control uses y-axis of left and right stick
 *   Gamepad2:  Lift controlled with dpad up and down
 *              Power offset controlled by button X
 *
 * Version 1.4
 */

@TeleOp ( name = "TeleOp for Competition")

public class TeleOpComp extends LinearOpMode {

    // private Gyroscope imu;
    public DcMotor  leftDrive;
    public DcMotor  rightDrive;
    public DcMotor  leftLift;
    public DcMotor  rightLift;
    public Servo    lockServo;
    public DigitalChannel digitalTouch;

    private Servo servoTest;

    @Override
    public void runOpMode () {
        // imu = hardwareMap.get(Gyroscope.class, "imu");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift = hardwareMap.get(DcMotor.class, "right_lift");

        lockServo = hardwareMap.servo.get("lift_lock_servo");

        digitalTouch = hardwareMap.digitalChannel.get("touch_sensor");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized - All Systems Go!");
        telemetry.update();

        // Wait for game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        double  left = 0;
        double  right = 0;
        int     liftArm = 0;
        double  powerOffset = 1;
        double  directionSwap = -1;
        double  liftPowerInit = 0.40;
        boolean touchState = true;
        boolean gamePadXState;
        boolean prevGamePadXState = false;

        double liftPower = liftPowerInit;

        while (opModeIsActive()) {
            // Use gamepad button x to toggle a power offset to drive motors
            gamePadXState = gamepad2.x;
            if ( gamePadXState ) {
                if (powerOffset == 1) {
                    powerOffset = 0.5;
                } else {
                    powerOffset = 1;
                }
            }

            // Use gamepad button b to set lift lock

            /**
             * if (gamepad2.b) {
             lockServo.setPosition(0.57);    // Locked Position
             }

             if (gamepad2.y) {
             lockServo.setPosition(0.24);    // Open Position
             }
             **/

            left = directionSwap * -this.gamepad1.left_stick_y;
            right = directionSwap * this.gamepad1.right_stick_y;
            leftDrive.setPower(left * powerOffset);
            rightDrive.setPower(right * powerOffset);

            // Control lift Arm - Stop motors when raising lift when touch sensor turns false
            touchState = digitalTouch.getState();

            if ( this.gamepad2.dpad_up && touchState ) {
                liftArm = 1;
                liftPower = liftPowerInit;
            } else if ( this.gamepad2.dpad_down ) {
                liftArm = -1;
                liftPower = liftPowerInit * 1.25;
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
            // telemetry.addData("Lift Power", liftPower);
            // telemetry.addData("Direction Swap", directionSwap );
            telemetry.addData("Touch Sensor", touchState );
            telemetry.addData("Status", "Running");
            telemetry.update();

            // Try to improve consistency of button X controlling speed offset
            if ( prevGamePadXState != gamePadXState ) { sleep(100); }
            prevGamePadXState = gamePadXState;
        }
    }
}
