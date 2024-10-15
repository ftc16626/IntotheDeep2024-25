package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp

public class IntoTheDeepArmAlt extends LinearOpMode {
    private DcMotor rotateArm;
    private DcMotor extendArm;


    @Override
    public void runOpMode() {
        rotateArm = hardwareMap.get(DcMotor.class, "rotateArm");
        rotateArm.setTargetPosition(0);
        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int rotTarget = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //MECHANISM CODE

            if (gamepad1.a) {
                ArmTarget = 0; //On the ground for starting and intaking
            }
            else if (gamepad1.x) {
                ArmTarget = 120; //Low level on the goal
            }
            else if (gamepad1.y) {
                ArmTarget = 260; //Mid level on the goal
            }
            else if (gamepad1.b) {
                ArmTarget = 410; //High level on the goal
            }
            else if (gamepad1.right_bumper) {
                ArmTarget = 1420; //High level on the goal scoring backwards
            }
            else if (gamepad1.left_bumper) {
                ArmTarget = 1570; //Mid level on the goal scoring backwards
            }

            //stuff for arm position control
            rotateArm.setTargetPosition(ArmTarget);
            rotateArm.setPower(1);

            telemetry.addData("Arm Position", ArmMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

