

package org.firstinspires.ftc.teamcode.outdated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="The Better Red Right With Arm", group="Shame")
@Disabled
public class RedRightWithArmDirectorscut extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         LFMotor   = null;
    private DcMotor         RFMotor  = null;
    private DcMotor         LBMotor   = null;
    private DcMotor         RBMotor  = null;
    private DcMotor         rotateArm = null;
    private DcMotor         extendArm = null;
    CRServo Wheel1;
    CRServo Wheel2;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV   = 537.7;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (PULLEY_DIAMETER_INCHES * 3.1415);
    static final double     ROTATE_GEAR_REDUC = 3.0 ;
    static final double     COUNTS_PER_DEGREE       = (COUNTS_PER_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     ROT_SPEED               = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        rotateArm = hardwareMap.get(DcMotor.class, "rotateArm");
        extendArm = hardwareMap.get(DcMotor.class, "extendArm");
        Wheel1 = hardwareMap.get(CRServo.class, "Wheel1");
        Wheel1.resetDeviceConfigurationForOpMode();
        Wheel2 = hardwareMap.get(CRServo.class, "Wheel2");
        Wheel2.resetDeviceConfigurationForOpMode();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        rotateArm.setDirection(DcMotor.Direction.REVERSE);
        extendArm.setDirection(DcMotor.Direction.REVERSE);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          LFMotor.getCurrentPosition(),
                          LBMotor.getCurrentPosition(),
                          RFMotor.getCurrentPosition(),
                          RBMotor.getCurrentPosition(),
                          rotateArm.getCurrentPosition(),
                          extendArm.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, ROT_SPEED,  21,  21, 0, 0, 0,0, 5.0);
        encoderDrive(DRIVE_SPEED, ROT_SPEED, 0, 0, 190, 0,0,0,5.0);
        encoderDrive(DRIVE_SPEED, ROT_SPEED, 0, 0, 0, 12.25,0,0,5.0);
        encoderDrive(.1, .1, 0, 0, -17, -2.25,-1,1,5.0);
        encoderDrive(.1, .1,  -5, -5,-90,-9,-1,1,  5.0);
        encoderDrive(TURN_SPEED, ROT_SPEED, -25, 25,0,0,0,0, 5.0);
        encoderDrive(DRIVE_SPEED, ROT_SPEED, 50, 50,0,0,0,0, 5.0);
        encoderDrive(DRIVE_SPEED, ROT_SPEED, 25, -25,0,0,0,0, 5.0);
        encoderDrive(DRIVE_SPEED, ROT_SPEED, 0, 0,190,0,0,0, 5.0);
        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


    public void encoderDrive(double speed, double armspeed,
                             double leftInches, double rightInches,
                             double RotDegrees, double ExtInches, double wheel1Power, double wheel2Power,
                             double timeoutS) {
        int newLFTarget;
        int newLBTarget;
        int newRFTarget;
        int newRBTarget;
        int newROTarget;
        int newEXTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = LFMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLBTarget = LBMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRFTarget = RFMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRBTarget = RBMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newROTarget = rotateArm.getCurrentPosition() + (int)(RotDegrees * COUNTS_PER_DEGREE);
            newEXTarget = extendArm.getCurrentPosition() + (int)(ExtInches * COUNTS_PER_EXTINCH);

            LFMotor.setTargetPosition(newLFTarget);
            RFMotor.setTargetPosition(newRFTarget);
            LBMotor.setTargetPosition(newLBTarget);
            RBMotor.setTargetPosition(newRBTarget);
            rotateArm.setTargetPosition(newROTarget);
            extendArm.setTargetPosition(newEXTarget);

            // Turn On RUN_TO_POSITION
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            LFMotor.setPower(Math.abs(speed));
            RFMotor.setPower(Math.abs(speed));
            LBMotor.setPower(Math.abs(speed));
            RBMotor.setPower(Math.abs(speed));
            rotateArm.setPower(Math.abs(armspeed));
            extendArm.setPower(Math.abs(speed));
            Wheel1.setPower(wheel1Power);
            Wheel2.setPower(wheel2Power);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            //onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (LFMotor.isBusy() || rotateArm.isBusy() || extendArm.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLFTarget, newRFTarget,  newRBTarget,  newLBTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            LFMotor.getCurrentPosition(), RFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition(), rotateArm.getCurrentPosition(), extendArm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LFMotor.setPower(0);
            RFMotor.setPower(0);
            LBMotor.setPower(0);
            RBMotor.setPower(0);
            rotateArm.setPower(.1);
            extendArm.setPower(0);
            Wheel1.setPower(0);
            Wheel2.setPower(0);

            // Turn off RUN_TO_POSITION
            LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);   // optional pause after each move.
        }
    }
}
