package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
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
            values.lowerAngle = atan2(y, x) + atan2((armLength2 * sin(abs(values.upperAngle))) , (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        } else {
            values.upperAngle = acos(cosAngle)
            values.lowerAngle = atan2(y, x) - atan2((armLength2 * sin(abs(values.upperAngle))) , (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
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
    val lowerAnglePID = PID(PIDSettings(kP = 0.0425, kI = 0.0, kD = 0.0))
    val upperAnglePID = PID(PIDSettings(kP = 0.0175, kI = 0.0, kD = 0.0001))
    private var scope = CoroutineScope(Job())


    lateinit var lowerMotor: DcMotor
    lateinit var upperMotor: DcMotor

    lateinit var pot1: AnalogInput
    lateinit var lowerAngle: AnalogInput
    lateinit var pot3: AnalogInput
    lateinit var upperAngle: AnalogInput

    lateinit var intakeLeft: DcMotor
    lateinit var intakeRight: DcMotor

    var upperAngleTarget: Double = 119.0
    var lowerAngleTarget: Double = 119.0

//    var targetX: Double = -0.2
//    var targetY: Double = 0.0
    var targetX: Double = -100.0
    var targetY: Double = 100.0

    val kinematics = ArmKinematics(armLenght1 = .26, armLength2 = .26)

    private var maxClampAngle = 0.0

    override fun init(hardwareMap: HardwareMap) {
        lowerMotor = hardwareMap.get(DcMotor::class.java, "lowerMotor")
        upperMotor = hardwareMap.get(DcMotor::class.java, "upperMotor")

        upperMotor.direction = DcMotorSimple.Direction.REVERSE

        intakeLeft = hardwareMap.get(DcMotor::class.java, "intakeLeft")
        intakeRight = hardwareMap.get(DcMotor::class.java, "intakeRight")

        pot1 = hardwareMap.get(AnalogInput::class.java, "analog1")
        lowerAngle = hardwareMap.get(AnalogInput::class.java, "lowerAngle")
        pot3 = hardwareMap.get(AnalogInput::class.java, "analog3")
        upperAngle = hardwareMap.get(AnalogInput::class.java, "upperAngle")


        lowerMotor.direction = DcMotorSimple.Direction.REVERSE

        intakeLeft.direction = DcMotorSimple.Direction.REVERSE
        intakeRight.direction = DcMotorSimple.Direction.REVERSE

    }

    fun moveto(cordinates: ArmCordinates) {
        var targetAngles = kinematics.calculateInversedKinematics(cordinates.x, cordinates.y)
    }

    fun getAngles(): ArmMotorValues {
        val angles = ArmMotorValues()

        angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0)
        val x = upperAngle.voltage
        angles.upperAngle = -1.66243031965927 * x.pow(3) - 9.090297864880776 * x.pow(2) + 130.359681271249 * x - 137.040643577643

        return angles
    }

    @ExperimentalCoroutinesApi
    override fun start() {
        scope.launch { lowerAnglePID.start() }
        scope.launch { upperAnglePID.start() }
        scope.launch { angleProducer() }
        scope.launch { angleReceiver() }
    }

    override fun stop() {
        scope.cancel()
        scope = CoroutineScope(Job())
    }

    suspend fun angleProducer() {
        while (scope.isActive) {
            val currentAngle = getAngles()
//            telemetry.addData("current angle", currentAngle)
            lowerAnglePID.inputChannel.send(currentAngle.lowerAngle)
            upperAnglePID.inputChannel.send(currentAngle.upperAngle)
        }
    }

    suspend fun angleReceiver() {
        while (scope.isActive) {

            val angles = kinematics.calculateInversedKinematics(targetX, targetY)

            val lowerTarget = (angles.lowerAngle * 180 / PI).coerceIn(55.0, 132.0)
            val upperTarget = (angles.upperAngle * 180 / PI).coerceIn(-135.0, 134.0)

            lowerAnglePID.target = lowerTarget
            upperAnglePID.target = upperTarget

            val currentAngles = getAngles()

            val targetCoord = kinematics.calculateFowardKinematics(currentAngles.lowerAngle * PI / 180, currentAngles.upperAngle * PI / 180)

            telemetry.addData("lowerTarget", targetCoord)

            val lowerOutput = lowerAnglePID.outputChannel.receive()
            val upperOutput = upperAnglePID.outputChannel.receive()
//            telemetry.addData("output:", lowerOutput)

//            lowerMotor.power = lowerOutput
//            upperMotor.power = upperOutput
        }
    }


    fun intake(power: Double) {
        intakeLeft.power = power
        intakeRight.power = power
    }

}
