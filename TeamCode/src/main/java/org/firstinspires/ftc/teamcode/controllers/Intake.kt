package org.firstinspires.ftc.teamcode.controllers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.controllers.motors.EctoDcMotor
import org.firstinspires.ftc.teamcode.core.Controller

class Intake : Controller() {
    val leftIntake = EctoDcMotor("intakeLeft")
    val rightIntake = EctoDcMotor("intakeRight")

    override fun init(hardwareMap: HardwareMap) {
        controllers.add(leftIntake)
        controllers.add(rightIntake)


    }

    var power = 0.0
        set(value) {
            field = value
            leftIntake.power = field * 0.7
            rightIntake.power = -field * 0.7
        }

    var rightPower = 0.0
        set(value) {
            field = value
            leftIntake.power = 0.0
            rightIntake.power = -field * 0.7
        }


    var leftPower = 0.0
        set(value) {
            field = value
            leftIntake.power = field * 0.7
            rightIntake.power = 0.0
        }
}