package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="DECODE_2025_Auto_Mecanum", group="DECODE")
public class StarterBotAuto extends OpMode {

    final double LAUNCH_TIME = 10.0;  // Fire balls for 10 seconds
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    int shotsToFire = 3;
    double robotRotationAngle = 45;

    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime totalLaunchTime = new ElapsedTime(); // For the 10 seconds of firing

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }

    private LaunchState launchState;

    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE;
    }

    private AutonomousState autonomousState;

    private enum Alliance {
        RED,
        BLUE;
    }

    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        autonomousState = AutonomousState.LAUNCH;
        launchState = LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);  // Mecanum setup
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);   // Mecanum setup
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Mecanum setup
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);  // Mecanum setup

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftRearDrive.setZeroPowerBehavior(BRAKE);
        rightRearDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);

        if (gamepad1.b) {
            alliance = Alliance.RED;
        } else if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void start() {
        totalLaunchTime.reset();  // Start the timer for the launch phase
    }

    @Override
    public void loop() {
        switch (autonomousState) {
            case LAUNCH:
                // Start launching for 10 seconds
                launch(true);
                totalLaunchTime.reset();  // Reset the launch timer
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                // Wait until the 10 seconds are up
                if (totalLaunchTime.seconds() >= LAUNCH_TIME) {
                    // Stop firing after 10 seconds
                    launch(false);
                    shotsToFire = 0;  // No more shots to fire
                    resetEncoders();
                    autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:
                // Move the robot back a certain distance (e.g., 12 inches)
                if (drive(DRIVE_SPEED, 0, -12, 3)) {
                    resetEncoders();
                    autonomousState = AutonomousState.ROTATING;
                }
                break;

            case ROTATING:
                // Rotate based on the alliance color
                if (alliance == Alliance.RED) {
                    robotRotationAngle = 45; // Rotate to a specific angle for red alliance
                } else if (alliance == Alliance.BLUE) {
                    robotRotationAngle = -45; // Rotate to a specific angle for blue alliance
                }

                if (rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 3)) {
                    resetEncoders();
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case DRIVING_OFF_LINE:
                // Drive forward to move off t
        }
    }
}
