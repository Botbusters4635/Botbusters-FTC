package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import java.math.RoundingMode
import java.text.DecimalFormat
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
    val lowerAnglePID = PID(PIDSettings(kP = 0.0, kI = 0.0, kD = 0.0))
    val upperAnglePID = PID(PIDSettings(kP = 0.0, kI = 0.0, kD = 0.0))
    private val scope = CoroutineScope(Job())


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

    fun roundToDecimals(number: Double, numDecimalPlaces: Int): Double {
        val factor = Math.pow(10.0, numDecimalPlaces.toDouble())
        return Math.round(number * factor) / factor
    }

    fun getAngles(): ArmMotorValues {
        val angles = ArmMotorValues()

        angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0)
        angles.upperAngle = (upperAngle.voltage - 1.65) / 3.3 * 180


        angles.upperAngle = 547.0 - 40.0 * angles.upperAngle + 0.895 * Math.pow(angles.upperAngle, 2.0) - 8.38 * Math.pow(10.0, -3.0) * Math.pow(angles.upperAngle, 3.0) + 2.92 * Math.pow(10.0, -5.0) * Math.pow(angles.upperAngle, 4.0)
//        547+-40x+0.895x^{2}-8.38\cdot10^{-3}x^{3}+2.92\cdot10^{-5}x^{4}
        telemetry.addData("lowerVoltage", lowerAngle.voltage)
        telemetry.addData("upperVoltage", upperAngle.voltage)

        return angles
    }

    override fun start() {
        scope.launch { lowerAnglePID.start() }
        scope.launch { upperAnglePID.start() }
        scope.launch { angleProducer()}
    }
    suspend fun angleProducer() {
        while (scope.isActive){
            val currentAngle = getAngles()
            lowerAnglePID.inputChannel.send(currentAngle.lowerAngle)
            upperAnglePID.inputChannel.send(currentAngle.upperAngle)
        }
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
