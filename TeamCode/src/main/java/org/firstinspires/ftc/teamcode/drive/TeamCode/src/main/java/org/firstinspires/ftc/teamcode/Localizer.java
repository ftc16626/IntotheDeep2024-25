package org.firstinspires.ftc.teamcode.drive.TeamCode.src.main.java.org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

public interface Localizer {
    Twist2dDual<Time> update();
}
