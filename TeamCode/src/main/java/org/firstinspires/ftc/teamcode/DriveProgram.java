package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MathEx.DEG2RAD;
import static org.firstinspires.ftc.teamcode.MathEx.boolToInt;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Drive Program", group = "Main")
public class DriveProgram extends OpMode {

    //variables for motors
//    DcMotor left_drive = null;
//    DcMotor right_drive = null;

    //variables for sensors
//    BNO055IMU gyro = null;
    BHI260IMU gyro = null;

    Omnidrive drive;
    DcMotor[] driveMotors;

//    private DcMotor downMotor;
    
    boolean brakeOn = false;

//    private DcMotor linearSlide;

    {
//        downMotor = null;
//        linearSlide = null;
    }

    private Servo hookServo;

    void init_motors() {
        int motorCount = 4;

        this.drive = new Omnidrive(motorCount);
        this.driveMotors = new DcMotor[motorCount];

        this.driveMotors[0] = this.hardwareMap.get(DcMotor.class, String.format("drive_%d", 0, 0.1f));
        this.driveMotors[1] = this.hardwareMap.get(DcMotor.class, String.format("drive_%d", 1, 0.1f));
        this.driveMotors[2] = this.hardwareMap.get(DcMotor.class, String.format("drive_%d", 2, 0.1f));
        this.driveMotors[3] = this.hardwareMap.get(DcMotor.class, String.format("drive_%d", 3, 0.1f));
//        linearSlide = hardwareMap.get(DcMotor.class, String.format("drive_%d", 0, 0.1f));
//        downMotor = hardwareMap.get(DcMotor.class, "down_motor");

//        linearSlide.setDirection(DcMotor.Direction.FORWARD);
//        downMotor.setDirection(DcMotor.Direction.REVERSE);

        this.hookServo = this.hardwareMap.get(Servo.class, "hook");
    }

    void init_sensors() {

        //init 9 axis gyroscope (built into controller)

        //options on how the gyro should work
//        BHI260IMU.Parameters parameters = new BNO055IMU.Parameters();
//        ImuOrientationOnRobot porient =
//        IMU.Parameters parameters = new IMU.Parameters(new ImuOrientationOnRobot());

        //how the angles should be represented (degrees or radians)
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        //get the gyro
        this.gyro = hardwareMap.get(BHI260IMU.class, "imu");

        this.gyro.initialize();
        //init with the options we want
//        this.gyro.initialize(parameters);

    }

    //controller left stick x,y
    private Vec2 inputDirection;
    //magnitude of left stick x,y
    private float inputMagnitude;

    private float desiredHeadingRadians;
    private float actualHeadingRadians;

    //stop buttons firing too fast to toggle
    private Debounce dAlign;
    //align mode, toggled with A button in loop()
    private boolean align;

    double linearPower = 0;
    double downPower;

    @Override
    public void init() {

        //create a vector to store our left stick x,y
        this.inputDirection = new Vec2();
        this.inputMagnitude = 0;

        //300 milliseconds between A being pressed before triggering again
        this.dAlign = new Debounce(300);

        //align mode, keeps to desired orientation (turn) when set to true
        //essentially compass based turning
        this.align = false;

        //where we want to be facing
        this.desiredHeadingRadians = 0;
        //where we're actually facing
        this.actualHeadingRadians = 0;

        //say hello
        this.telemetry.addData("main", "Hello World");
        this.telemetry.update();

        //init motors so we can use them later
        this.init_motors();

        //init sensors we read from
        this.init_sensors(); //temporarily disable until we figure out what type of IMU this control hub has
    }

//    Orientation o;

    void update_sensors() {
//        this.o = this.gyro.getAngularOrientation();
        YawPitchRollAngles angles = this.gyro.getRobotYawPitchRollAngles();

        this.actualHeadingRadians = (float) angles.getYaw(AngleUnit.RADIANS);//this.o.firstAngle;
    }

    float driveDirectionRadians = 0;
    float headingTolerance = MathEx.DEG2RAD * 5f;
    float headingAdjust = 0;
    float turnRaw = 0;
    float rad360 = MathEx.DEG2RAD * 360;

    Debounce dPerpTurn = new Debounce(200);

    Debounce dHookWait = new Debounce(350);
    double hookOpenedPosition = 0.5d;
    double hookClosedPosition = 1d;
    double hookTargetPosition = 0.5d;

    boolean hookShouldBeClosed = true;

    void handle_input() {

        //DRIVING DIRECTION / SPEED
        //copy stick direction
        this.inputDirection.set(
                -this.gamepad1.left_stick_x,
                this.gamepad1.left_stick_y
        );

        //calculate magnitude of direction
        this.inputMagnitude = this.inputDirection.magnitude();

        //get rid of magnitude from actual direction vector
        this.inputDirection.normalize();

        this.driveDirectionRadians = this.inputDirection.arctan2();

        //offset angle by -45 deg
        this.driveDirectionRadians += (45f * DEG2RAD);

        //HEADING / TURNING
        this.turnRaw = this.gamepad1.right_stick_x;

        if (Float.isNaN(this.turnRaw)) this.turnRaw = 0.0f; //TODO - check if necessary

        //change desired heading based on joystick
        this.desiredHeadingRadians -= this.turnRaw / 25.0f; //divide by 25 to slow the rate down, TODO - make adjustable
        this.desiredHeadingRadians %= MathEx.DEG2RAD * 360;

        //left/right bumper change heading angle by 90deg
        int turn = boolToInt(this.gamepad1.right_bumper) - boolToInt(this.gamepad1.left_bumper);
        if (turn != 0 && this.dPerpTurn.update()) {
            this.desiredHeadingRadians -= (90f * turn)*DEG2RAD;
        }

        //calculate align mode heading (even if not used while updating motors)
        this.headingAdjust = 0;
        float headingDiff = (float) Math.atan2(
                Math.sin(
                        this.desiredHeadingRadians - this.actualHeadingRadians
                ),
                Math.cos(this.desiredHeadingRadians - this.actualHeadingRadians)
        );
        if (Math.abs(headingDiff) > headingTolerance) this.headingAdjust = (headingDiff / rad360) * 2;
        if (this.headingAdjust < -1) this.headingAdjust = -1;
        if (this.headingAdjust > 1) this.headingAdjust = 1;

        //toggle align mode based on gamepad A button and debounce settings
        if (this.gamepad1.a && this.dAlign.update()) {
            this.align = !this.align;
            telemetry.addData("Align mode", "%b", this.align);
        }
        //Linear Slide
        if(gamepad2.left_trigger==1&&gamepad2.right_trigger==0) {
            linearPower = 1;
        }
        else if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 1) {
            linearPower = 0;

        } else {
            linearPower = -1;
        }
        //log stuff if in align mode
        if (this.align) {
            this.telemetry.addData("Heading", "Desired: %.2f, Actual: %.2f, Adjust: %.2f",
                    this.desiredHeadingRadians,
                    this.actualHeadingRadians,
                    headingAdjust
            );
        }

        //check the gamepad as well as the timeout, flip the boolean to be the opposite
        if (this.gamepad1.b && this.dHookWait.update()) {
            this.hookShouldBeClosed = !this.hookShouldBeClosed;
        }

        double hookCurrentPosition = this.hookServo.getPosition();

        this.telemetry.addData("Hook", "Current position %f , target: %f", hookCurrentPosition, this.hookTargetPosition);

        if (this.hookShouldBeClosed) {
            this.hookTargetPosition = this.hookClosedPosition;
        } else {
            this.hookTargetPosition = this.hookOpenedPosition;
        }
        this.hookServo.setPosition(this.hookTargetPosition);
    }

    public void setBrake() {
        boolean breakOn = false;
        if(!breakOn) {
//            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            downMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            brakeOn = true;

        }else{
//            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            downMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            brakeOn = false;
        }
    }

    void update_motors() {
        //send orientation as text so we can see it
//        this.telemetry.addData(
//                "Orientation", "{ x: %.2f, y: %.2f, z: %.2f }",
//                o.firstAngle,
//                o.secondAngle,
//                o.thirdAngle
//        );

        //calculate drive.motorOutput array
        this.drive.update(
                this.driveDirectionRadians, this.inputMagnitude
        );

        //store variable for motor power
        float motorPower = 0;

        //apply drive calculations to physical motors
        for (int i = 0; i < this.drive.motorOutput.length; i++) {

            //if in align mode, add motor output to headingAdjust so we can match gyro
//            this.align = false; //TODO - remove if using IMU
            if (this.align) {
                motorPower = (this.drive.motorOutput[i] / 2) + this.headingAdjust;
                if (Float.isNaN(motorPower)) motorPower = this.headingAdjust;

            } else { //otherwise just bias the motor output with the raw turn value from the joystick
                motorPower = (this.drive.motorOutput[i] / 2) - (this.turnRaw / 2);
                if (Float.isNaN(motorPower)) motorPower = -(this.turnRaw / 2);
            }

            //finally output the motor power to the specific motor
            this.driveMotors[i].setPower(motorPower);
//            linearSlide.setPower(linearPower);
//            downMotor.setPower(downPower);

        }
    }

    @Override
    public void loop() {
        //read from sensors (gyro)
        this.update_sensors();

        //read and calculate gamepad inputs
        this.handle_input();

        //output to motors
        this.update_motors();

        //anything that needs logged will now do so
        this.telemetry.update();

    }
}