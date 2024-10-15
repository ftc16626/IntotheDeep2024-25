package org.firstinspires.ftc.teamcode;
// TEST1 Oct Alex

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="HighFiveTest", group="TestCode")
public class IntoTheDeepArm extends LinearOpMode {
    private DcMotor extendArm;
    private DcMotor rotateArm;
    static final double     COUNTS_PER_MOTOR_REV   537.7 =  ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     COUNTS_PER_GEAR         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);
/* Define motors in DriverHub */
    @Override
    public void runOpMode() {
        extendArm = hardwareMap.get(DcMotor.class,"extendArm");
        rotateArm = hardwareMap.get(DcMotor.class,"rotateArm");

        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

public void encoderArm(double speed, double position) {
    int newRotTarget;
    int newExtTarget;
    double rotate = ga
    if (opModeIsActive()) {

        newRotTarget = rotateArm.getCurrentPosition +
    }
        }


        while( opModeIsActive()){

                        }
        }
    }

