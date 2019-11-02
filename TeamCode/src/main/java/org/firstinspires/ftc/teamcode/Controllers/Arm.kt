package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Core.Controller
import kotlin.math.*

data class ArmMotorValues(var angle1: Double = 0.00, var angle2: Double = 0.00)

class ArmKinematics(val armLenght1: Double, val armLength2: Double) {


    fun calculateInversedKinematics(x: Double, y: Double): ArmMotorValues {
        val values = ArmMotorValues()
        values.angle2 = acos((x.pow(2) + y.pow(2) - armLenght1.pow(2) - armLength2.pow(2)) / (2 * armLenght1 * armLength2))
        values.angle1 = atan(y / x) - atan(armLength2 * sin(values.angle2) / (armLenght1 + armLength2 * cos(values.angle2)))
        return values


    }
}


class Arm : Controller() {
    lateinit var armMotor1: DcMotor
    lateinit var armMotor2: DcMotor
    val kinematics = ArmKinematics(armLenght1 = 26.0, armLength2 = 26.0

    )

    override fun init(hardwareMap: HardwareMap) {
        armMotor1 = hardwareMap.get(DcMotor::class.java, "armMotor1")
        armMotor2 = hardwareMap.get(DcMotor::class.java, "armMotor2")
    }


}