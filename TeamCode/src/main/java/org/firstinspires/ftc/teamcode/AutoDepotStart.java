package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto Depot Start")

// @Disabled

public class AutoDepotStart extends LinearOpMode {

    /* Declare OpMode members. */

    HardwareMain robot = new HardwareMain();

    private ElapsedTime runtime = new ElapsedTime();

    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .60, correction;

    // Create static variables
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.8;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.3;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                          (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double deployServoPosition = 0.7;
    static final double openLiftLockServoPosition = 0.9;
    static final double resetServoPosition = 0;
    static final double liftPower = 0.4;
    static boolean touchState = true;
    static int down = -1;   // Reverses direction of motors to lower lift

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);     // pause for telemetry msg to be seen

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Version", "0.2");
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // TODO: Init gyro - should set to zero (? validate this)
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Start autonomous actions here
        // Path: Launch from Depot side

        // Action 0:
        // Open lift lock servo
        robot.lockServo.setPosition(openLiftLockServoPosition);
        sleep(400);

        // ACTION 1:
        // Raise lift till stopped by touch sensor
        touchState = robot.digitalTouch.getState();
        while ( opModeIsActive() && touchState == true ) {
            robot.leftLift.setPower(liftPower * 0.9);
            robot.rightLift.setPower(-liftPower * 0.9);
            touchState = robot.digitalTouch.getState();
            telemetry.addData("Touch Sensor", touchState );
            telemetry.update();
        }

        // Turn off lift motors
        robot.leftLift.setPower(0);
        robot.rightLift.setPower(0);
        sleep(100);

        // ACTION 2:
        // Back up 4 inches from lander (doesn't need gyro)
        encoderDrive(DRIVE_SPEED,  -4,  -4, 2.0);  // Backwards X Inches with X Sec timeout

        // Lower lift - Lower for 1 second, will not get all the way down
        runtime.reset();        // reset the timeout time
        while ( opModeIsActive() && (runtime.seconds() < 1) ) {
            robot.leftLift.setPower(down * liftPower * 0.7);
            robot.rightLift.setPower(down * -liftPower * 0.7);
            telemetry.addData("Lowering lift for", "1 second");
            telemetry.addData("Timer:", runtime.seconds() );
            telemetry.update();
        }

        // Turn off lift motors
        robot.leftLift.setPower(0);
        robot.rightLift.setPower(0);

        // ACTION:  TODO
        // Realign to initial gyro heading (0 ?)
        //       Rotate to zero

        // ACTION 3:
        // Drive to depot using multiple drive and rotate steps
        encoderDrive(DRIVE_SPEED,  -12,  -12, 4.0);
        sleep(100);
        rotate(55, TURN_SPEED, 3.0);
        sleep(100);
        encoderDrive(DRIVE_SPEED, -33, -33, 6.0);
        sleep(100);
        rotate(220, TURN_SPEED, 5.0);
        sleep(100);
        encoderDrive(DRIVE_SPEED,  -37,  -37, 4.0);

        // May want to add longer delay to avoid partner in depot

        // ACTION 4:
        // Run servo to place marker and claim depot
        robot.markerServo.setPosition(deployServoPosition);

        sleep(100);

        telemetry.addData("Back Up", robot.markerServo.getPosition());
        telemetry.update();
        // ACTION 5:
        // Drive from depot to the crater
        encoderDrive(DRIVE_SPEED, 72, 72, 8.0);

        // ACTION 6:
        // Reset servo to upright position
        robot.markerServo.setPosition(resetServoPosition);

        // This completes phase 1 goals, make sure above works consistantly before
        // building any additional phase 2 goals

        // End autonomous actions here

        telemetry.addData("Autonomous Path", "Complete");
        telemetry.update();

        sleep(500);
    }

    public void gyroDrive(double speed,
                          double leftInches, double rightInches,
                          double timeoutS) {
        /* Drive in a straight line using gyro to maintain heading. */

        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        // reset the timeout time and start motion.
        runtime.reset();

        robot.leftDrive.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));

        /* Stay in while look and keep driving when:
              - OpMode is running AND
              - Timer hasn't expired AND
              - Left OR right motors encoder returned position hasn't exceeded
                the relevant target
         */
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.getCurrentPosition() < newLeftTarget ||
                     robot.leftDrive.getCurrentPosition() < newRightTarget)) {

            correction = checkDirection();

            robot.leftDrive.setPower(speed + correction);
            robot.rightDrive.setPower(speed);

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);

            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
            telemetry.update();

        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            telemetry.addData("Autonomous", "Ready to Start");
            sleep(100);
            telemetry.addData("Inches", leftInches);
            telemetry.addData("Tics/inch", COUNTS_PER_INCH);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition());
            telemetry.update();

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading
        // angle. We have to process the angle because the imu works in euler angles so
        //  the Z axis is returned as 0 to +180 or 0 to -180 rolling back to -179 or
        // +179 when rotation passes 180 degrees. We detect this transition and track
        // the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        telemetry.addData("Angle is ", globalAngle);
        telemetry.update();

        sleep(50);
        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction
        // changes to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than
     * 180 degrees. @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power, double timeoutS)
    {
        double  leftPower, rightPower;

        runtime.reset();        // reset the timeout time

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        telemetry.addData("Motors On. Degrees: ", degrees);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            telemetry.addData("Turning", "Right");
            telemetry.update();

            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0
                   && (runtime.seconds() < timeoutS)) {}

            while (opModeIsActive() && getAngle() > degrees
                   && (runtime.seconds() < timeoutS)) {}
        }
        else    // left turn.
            telemetry.addData("Turning", "Left");
            telemetry.update();

            while (opModeIsActive() && getAngle() < degrees
                   && (runtime.seconds() < timeoutS)) {
                //telemetry.addData("Turning Left",  "Angle is %7d ", getAngle());
                //telemetry.update();
            }

        // turn the motors off.
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void rotateToDegree(int degrees, double power, double timeoutS)
    {
        // TODO: Needs testing!!!!
        // Does not support turning to 0 degrees.  If 0 sent currently does nothing.

        double  leftPower, rightPower;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0
                   && (runtime.seconds() < timeoutS)) {}

            while (opModeIsActive() && getAngle() > degrees
                   && (runtime.seconds() < timeoutS)) {
                telemetry.addData("Turning", "Right");
                telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees
                   && (runtime.seconds() < timeoutS)) {
                //telemetry.addData("Turning", "Left");
                //telemetry.update();
            }

        // turn the motors off.
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // wait for rotation to stop.
        sleep(750);
    }
}
