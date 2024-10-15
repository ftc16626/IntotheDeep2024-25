package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class TestClaw2 extends LinearOpMode{
    CRServo Wheel1;
    CRServo Wheel2;
    public void runOpMode(){
        Wheel1 = hardwareMap.get(CRServo.class, "Wheel1");
        Wheel1.resetDeviceConfigurationForOpMode();
        Wheel2 = hardwareMap.get(CRServo.class, "Wheel2");
        Wheel2.resetDeviceConfigurationForOpMode();
        waitForStart();

        while(opModeIsActive()){
                while (gamepad1.dpad_up) {
                    Wheel1.setPower(0.6);
                    Wheel2.setPower(-0.8);
                }
                while (gamepad1.dpad_down) {
                    Wheel1.setPower(-0.6);
                    Wheel2.setPower(0.8);
            }
                while (gamepad1.dpad_left){
                    Wheel1.setPower(0.0);
                    Wheel2.setPower(0.0);
                }
        }
    }
}

