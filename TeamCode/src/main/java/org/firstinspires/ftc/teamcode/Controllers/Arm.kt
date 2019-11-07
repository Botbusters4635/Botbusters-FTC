package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.core.Controller
import kotlin.math.*

data class ArmMotorValues(var lowerAngle: Double = 0.00, var upperAngle: Double = 0.00)
data class ArmCordinates(var x: Double = 0.00, var y: Double = 0.00)

class ArmKinematics(val armLenght1: Double, val armLength2: Double) {


    fun calculateInversedKinematics(x: Double, y: Double): ArmMotorValues {
        val values = ArmMotorValues()
        var cosAngle = (x.pow(2) + y.pow(2) - armLenght1.pow(2) - armLength2.pow(2)) / (2 * armLenght1 * armLength2)

        if (cosAngle > 1) {
            cosAngle = 1.0
        } else if (cosAngle < -1.0) {
            cosAngle = -1.0
        }


        if (x > 0) {
            values.upperAngle = -acos(cosAngle)
            values.lowerAngle = atan(y / x) + atan((armLength2 * sin(abs(values.upperAngle))) / (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        } else {
            values.upperAngle = acos(cosAngle)
            values.lowerAngle = atan(y / x) - atan((armLength2 * sin(abs(values.upperAngle))) / (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        }
        return values

    }

    fun calculateFowardKinematics(angle1: Double, angle2: Double): ArmCordinates {
        val values2 = ArmCordinates()
        values2.x = cos(angle1) * armLenght1 + cos(angle1 + angle2) * armLength2
        values2.y = sin(angle1) * armLenght1 + sin(angle1 + angle2) * armLength2

        return values2
    }


}


class Arm : Controller() {

    lateinit var armMotor1: DcMotor
    lateinit var armMotor2: DcMotor

    lateinit var pot1: AnalogInput
    lateinit var lowerAngle: AnalogInput
    lateinit var pot3: AnalogInput
    lateinit var upperAngle: AnalogInput

    lateinit var intakeLeft: DcMotor
    lateinit var intakeRight: DcMotor

    val kinematics = ArmKinematics(armLenght1 = .26, armLength2 = .26)

    private var maxClampAngle = 0.0

    override fun init(hardwareMap: HardwareMap) {
        armMotor1 = hardwareMap.get(DcMotor::class.java, "armMotor1")
        armMotor2 = hardwareMap.get(DcMotor::class.java, "armMotor2")
        intakeLeft = hardwareMap.get(DcMotor::class.java, "intakeLeft")
        intakeRight = hardwareMap.get(DcMotor::class.java, "intakeRight")

        pot1 = hardwareMap.get(AnalogInput::class.java, "analog1")
        lowerAngle = hardwareMap.get(AnalogInput::class.java, "lowerAngle")
        pot3 = hardwareMap.get(AnalogInput::class.java, "analog3")
        upperAngle = hardwareMap.get(AnalogInput::class.java, "upperAngle")


        armMotor1.direction = DcMotorSimple.Direction.REVERSE

        intakeLeft.direction = DcMotorSimple.Direction.REVERSE
        intakeRight.direction = DcMotorSimple.Direction.REVERSE

    }

    fun moveto(cordinates: ArmCordinates) {
        var targetAngles = kinematics.calculateInversedKinematics(cordinates.x, cordinates.y)
    }

    fun getAngles(): ArmMotorValues {
        val angles = ArmMotorValues()

        // Change to radians/sec instead of ticks
        angles.lowerAngle = (2.891 - lowerAngle.voltage) / 2.285 * 90
        //angles.upperAngle = ((upperAngle.voltage - 1.62) / 0.163 * 90) - angles.lowerAngle * 1.25
        val x = angles.lowerAngle
        angles.upperAngle = /*((upperAngle.voltage - 1.658) / 0.186 * 90) - */-((0.006732198142415*x.pow(2)+0.312061403508772*x+4.09494324045409) + angles.lowerAngle)

        return angles
    }


    fun intake(power: Double) {
        intakeLeft.power = power
        intakeRight.power = power
    }

    fun writeMotors() {
        armMotor1.power = 0.0
        armMotor2.power = 0.2
    }

}
