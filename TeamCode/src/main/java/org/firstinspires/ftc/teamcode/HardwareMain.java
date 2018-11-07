package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define the specific hardware for the new_chassis.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Left  drive motor:        "left_lift"
 * Motor channel:  Right drive motor:        "right_lift"
 * Servo:          Marker placing servo:     "marker_servo"

 */

public class HardwareMain {

    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftLift   = null;
    public DcMotor  rightLift  = null;
    public Servo    markerServo;
    public BNO055IMU       imu;

    public static final double MID_SERVO         = 0.5;
    public static final double servoInitPosition = 0.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMain(){

    }

    /* Initialize standard Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        // Drive Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        // leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Lift Motors
        leftLift = hwMap.get(DcMotor.class, "left_lift");
        rightLift = hwMap.get(DcMotor.class, "right_lift");

        // Gyro onboard Rev Hub
        imu = hwMap.get(BNO055IMU.class, "imu");

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        markerServo = hwMap.servo.get("marker_servo");
        markerServo.setPosition(servoInitPosition);

    }
}
