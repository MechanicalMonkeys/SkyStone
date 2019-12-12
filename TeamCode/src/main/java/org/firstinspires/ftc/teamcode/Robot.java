package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

// THIS IS NOT AN OPMODE - IT IS A DEFINING CLASS
public class Robot {

    // Units
    RobotDrive drive;
    RobotArm arm;
    //TensorflowModel tfmodel;

    // Motors
    DcMotor rearLeft;
    DcMotor rearRight;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor waffleMover;
    DcMotor armRotate;
    DcMotor liftMotor;

    // Servos
    Servo gripperRotateServo, grabServo, capstoneArm;

    // Sensors
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    BNO055IMU imu;

    DistanceSensor frontDistance, leftDistance;
    ModernRoboticsI2cRangeSensor leftRange, frontRange;
    ColorSensor insideColor;
    TouchSensor rearTouch;

    // Vuforia
    WebcamTest detector;

    // Constants
    private int TORQUENADO60TICKS_PER_REV = 1440; // ticks / rev
    private int ANGLE_OF_GRIPPER_WHEN_GRABBING = 30; // in degrees
    private double TORQUENADO20_TICKS_PER_REV = 480; // ticks / rev
    private double WHEEL_DIAMETER = 4;
    private double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // in / rev
    double TICKS_PER_INCH = TORQUENADO20_TICKS_PER_REV / WHEEL_CIRCUMFERENCE; // ticks / in
    double ROBOT_EXTENDED_LENGTH = 30.0; // in
    double ROBOT_RETRACTED_LENGTH = 18.0; // in

    // PID Wrist Controller
    double pWrist = 0.0;
    double iWrist = 0.0;
    double dWrist = 0.0;
    PIDController PIDWrist = new PIDController(pWrist, iWrist, dWrist);

    // info
    int wafflePosition = -1; // 1 = Up, -1 = Down Waffle mover starts down
    double wafflePower = 0.5;

    double gripperRotatePosition = 1.0; // 1.0 = at a 90 degree angle, 0.9 = parallel to ground

    double capstoneArmPosition = 1.0;

    enum gripperPosition {OPEN, CLOSED}
    gripperPosition gripperPos = gripperPosition.OPEN;

    enum armPosition {REST, ACTIVE}
    armPosition armPos = armPosition.REST;
    Orientation angles;

    private HardwareMap hwMap = null;

    public Robot () {
        // Constructor
    }

    void init (OpMode opmode, boolean... doTF) {
        /* Initializes the robot */

        hwMap = opmode.hardwareMap;

        // Motor mapping
        this.rearLeft = hwMap.get(DcMotor.class, "rearLeft");
        this.frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        this.rearRight = hwMap.get(DcMotor.class, "rearRight");
        this.frontRight = hwMap.get(DcMotor.class, "frontRight");
        this.waffleMover = hwMap.get(DcMotor.class, "waffleMover");
        this.armRotate = hwMap.get(DcMotor.class, "armRotate");
        this.liftMotor = hwMap.get(DcMotor.class, "liftMotor");

        // Drive Motor Direction
        this.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        this.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.rearRight.setDirection(DcMotor.Direction.REVERSE);
        this.frontRight.setDirection(DcMotor.Direction.REVERSE);
        this.waffleMover.setDirection(DcMotor.Direction.FORWARD);
        this.armRotate.setDirection(DcMotor.Direction.REVERSE); // positive makes arm go forward
        this.liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Motor encoder reset
        this.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servo mapping
        this.gripperRotateServo = hwMap.get(Servo.class, "gripperRotateServo");
        this.grabServo = hwMap.get(Servo.class, "grabServo");
        this.capstoneArm = hwMap.get(Servo.class, "capstoneArm");

        // Servo direction
        this.gripperRotateServo.setDirection(Servo.Direction.FORWARD);
        this.grabServo.setDirection(Servo.Direction.FORWARD);
        this.capstoneArm.setDirection(Servo.Direction.FORWARD);

        // Sensor init
        this.frontDistance = hwMap.get(DistanceSensor.class, "frontDistance");
        this.leftDistance = hwMap.get(DistanceSensor.class, "leftDistance");
        this.leftRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
        this.frontRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "frontRange");


        //this.leftColor = hwMap.get(ColorSensor.class, "leftColor");
        this.insideColor = hwMap.get(ColorSensor.class, "insideColor");

        this.rearTouch = hwMap.get(TouchSensor.class, "rearTouch");

        // Units
        this.drive = new RobotDrive(this, this.rearLeft, this.rearRight, this.frontLeft, this.frontRight);
        this.arm = new RobotArm(this, this.armRotate, this.gripperRotateServo, this.grabServo);
//        if (doTF != null) {
//            if (doTF[0]) {
//                this.tfmodel = new TensorflowModel(hwMap);
//            }
//        }

        // set motor powers to 0 so they don't cause problems because thats what we do bruh moment :)
        this.drive.stopDrive();
        this.waffleMover.setPower(0);
        this.arm.armRotate.setPower(0);
        this.liftMotor.setPower(0);

        // init imu
        this.initImu();

        // Vuforia init
        if (doTF[0]) {
            detector = new WebcamTest();
            detector.init(hwMap);
        }

        // NavX Gyro Init
        //this.initNavXGyro(opmode);

    }

    void moveWaffleMover(boolean hold) {
        this.waffleMover.setPower(this.wafflePower * this.wafflePosition);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (true) {
            if (timer.time(TimeUnit.MILLISECONDS) > 800) {
                break;
            }
        }
        if (!hold) {
            this.waffleMover.setPower(0);
        } else {
            this.waffleMover.setPower(0.35);
        }
        this.wafflePosition *= -1;
    }

    void stopWaffleMover() {
        this.waffleMover.setPower(0);
    }

    void liftUp() { this.liftMotor.setPower(0.5); }

    void liftDown() { this.liftMotor.setPower(-0.5); }

    void stopLift() { this.liftMotor.setPower(0); }

    int detectSkystone(OpMode opmode) {
        return detector.detectSkystonePosition(opmode);
    }

    String getInfo() {
        String output = "Arm Position: " + this.armRotate.getCurrentPosition() + "\nWaffle Position: ";
        if (this.wafflePosition == -1) {
            output += "Down\nWrist Position: ";
        } else {
            output += "Up\nWrist Position: ";
        }

        output += this.gripperRotateServo.getPosition() + "\nGripper Position: ";

        output += this.gripperPos;

        output += "\nRobot Angle: " + this.getHeading();

        output += "\nRed: " + this.insideColor.red();

        output += "\nCapstone Arm: " + this.capstoneArmPosition;

        return output;
    }

    void initNavXGyro(OpMode opmode) throws InterruptedException {
        // A timer helps provide feedback while calibration is taking place
        ElapsedTime timer = new ElapsedTime();

        // Get a reference to a Modern Robotics GyroSensor object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        navxMicro = opmode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;
        // If you're only interested in the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "navx");

        // The gyro automatically starts calibrating. This takes a few seconds.
        opmode.telemetry.log().add("Gyro Calibrating. DO NOT MOVE or else the Cookie Monster will eat your soul!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating())  {
            opmode.telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|" : "-");
            opmode.telemetry.update();
            Thread.sleep(50);
        }
        opmode.telemetry.log().clear(); opmode.telemetry.log().add("Gyro Calibrated. Press Start.");
        opmode.telemetry.clear(); opmode.telemetry.update();

    }

    double getAngle() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    void stopEverything() {
        this.drive.stopDrive();
        this.arm.stopArmRotate();
        this.stopWaffleMover();
        this.stopLift();
    }

    void initImu() {
        // imu init
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        this.imu = hwMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
    }

    double getHeading() {
        // get angles
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    void toggleCapstoneArm() {
        this.capstoneArmPosition = 1 - this.capstoneArmPosition;
        capstoneArm.setPosition(this.capstoneArmPosition);
    }

    void initTFLite() throws IOException {
//        tfmodel.initTFLite();
    }

    void closeTFLite() {
//        tfmodel.closeInterpreter();
    }

    int detectImage() throws InterruptedException {
//        float[] prediction = tfmodel.predictPos();
//        float max_value = -1; // no prediction will be negative
//        int index_of_max = 0;
//        for(int i = 0; i < 3; i++) {
//            if (prediction[i] > max_value) {
//                max_value = prediction[i];
//                index_of_max = i;
//            }
//        }
//        return index_of_max;
//        //return 4;
        return -4;
    }
}

