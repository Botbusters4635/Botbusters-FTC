package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Core.Controller
import kotlin.math.*

data class ArmMotorValues(var angle1: Double = 0.00, var angle2: Double = 0.00 )
data class ArmCordinates(var x: Double = 0.00, var y: Double = 0.00)

class ArmKinematics(val armLenght1: Double, val armLength2: Double) {


    fun calculateInversedKinematics(x: Double, y: Double): ArmMotorValues {
        val values = ArmMotorValues()
        var cosAngle = (x.pow(2) + y.pow(2) - armLenght1.pow(2) - armLength2.pow(2))/ (2* armLenght1 * armLength2)

        if (cosAngle > 1 ){
            cosAngle = 1.0
        }else if (cosAngle < -1.0){
            cosAngle = -1.0
        }


        if (x > 0){
            values.angle2 = -acos(cosAngle)
            values.angle1 = atan(y/x) + atan((armLength2 * sin(abs(values.angle2))) / (armLenght1 + armLength2 * cos(abs(values.angle2))))
        } else {
            values.angle2 = acos(cosAngle)
            values.angle1 = atan(y/x) - atan((armLength2 * sin(abs(values.angle2))) / (armLenght1 + armLength2 * cos(abs(values.angle2))))
        }
        return values

    }

    fun calculateFowardKinematics(angle1: Double, angle2: Double): ArmCordinates{
        val values2 = ArmCordinates()
        values2.x = cos(angle1) * armLenght1 + cos(angle1 + angle2) * armLength2
        values2.y = sin(angle1) * armLenght1 + sin(angle1 + angle2) * armLength2

        return values2
    }


}



class Arm : Controller() {

    lateinit var armMotor1: DcMotor
    lateinit var armMotor2: DcMotor

    val kinematics = ArmKinematics(armLenght1 = .26, armLength2 = .26)

    override fun init(hardwareMap: HardwareMap) {
        armMotor1 = hardwareMap.get(DcMotor::class.java, "armMotor1")
        armMotor2 = hardwareMap.get(DcMotor::class.java, "armMotor2")
    }

    fun moveto(cordinates: ArmCordinates) {
        var targetAngles = kinematics.calculateInversedKinematics(cordinates.x, cordinates.y)
    }
    fun getAngles(){
        val angles = ArmMotorValues()


        angles.angle1 = 0.0
        angles.angle2 = 0.0
    }

}
