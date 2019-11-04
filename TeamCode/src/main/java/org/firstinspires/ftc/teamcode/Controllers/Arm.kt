package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.core.Controller
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

    lateinit var intakeLeft: DcMotor
    lateinit var intakeRight: DcMotor
    lateinit var turningServo: CRServo
    lateinit var clampServo: CRServo

    val kinematics = ArmKinematics(armLenght1 = .26, armLength2 = .26)

    private var maxClampAngle = 0.0

    override fun init(hardwareMap: HardwareMap) {
        armMotor1 = hardwareMap.get(DcMotor::class.java, "armMotor1")
        armMotor2 = hardwareMap.get(DcMotor::class.java, "armMotor2")
        intakeLeft = hardwareMap.get(DcMotor::class.java, "intakeLeft")
        intakeRight = hardwareMap.get(DcMotor::class.java, "intakeRight")
        turningServo = hardwareMap.get(CRServo::class.java, "turningServo")
        clampServo = hardwareMap.get(CRServo::class.java, "clampServo")

        armMotor1.direction = DcMotorSimple.Direction.REVERSE

        intakeLeft.direction = DcMotorSimple.Direction.REVERSE
        intakeRight.direction = DcMotorSimple.Direction.REVERSE

    }

    fun moveto(cordinates: ArmCordinates) {
        var targetAngles = kinematics.calculateInversedKinematics(cordinates.x, cordinates.y)
    }
    fun getAngles(){
        val angles = ArmMotorValues()


        angles.angle1 = 0.0
        angles.angle2 = 0.0
    }

    fun setClawHeading(heading : Double){
        turningServo.power = heading
    }

    fun setClampForce(power : Double){
        var outputPower = power

        if(outputPower > 1.0){
            outputPower = 1.0
        }else if(outputPower < -1.0){
            outputPower = -1.0
        }

        clampServo.power = maxClampAngle * outputPower
    }

    fun intake(power : Double){
        intakeLeft.power = power
        intakeRight.power = power
    }

    fun writeMotors(){
        armMotor1.power = 0.0
        armMotor2.power = 0.2
    }

}
