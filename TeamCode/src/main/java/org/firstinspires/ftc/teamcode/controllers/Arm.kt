package org.firstinspires.ftc.teamcode.controllers

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import kotlin.math.*

data class ArmMotorValues(var lowerAngle: Double = 0.00, var upperAngle: Double = 0.00)


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
            values.lowerAngle = atan2(y, x) + atan2((armLength2 * sin(abs(values.upperAngle))), (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        } else {
            values.upperAngle = acos(cosAngle)
            values.lowerAngle = atan2(y, x) - atan2((armLength2 * sin(abs(values.upperAngle))), (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        }
        return values

    }

    fun calculateFowardKinematics(angle1: Double, angle2: Double): Coordinate {
        val values2 = Coordinate()
        values2.x = cos(angle1) * armLenght1 + cos(angle1 + angle2) * armLength2
        values2.y = sin(angle1) * armLenght1 + sin(angle1 + angle2) * armLength2

        return values2
    }


}

enum class ArmState {
    TARGET_LOW, TARGET_HIGH, EXCHANGE
}

enum class ArmPosition(val coordinate: Coordinate) {
    HOME(Coordinate(0.22, 0.1)), TOP(Coordinate(-0.26, 0.4)), MEDIUM(Coordinate(-0.26, 0.2)), LOW(Coordinate(-0.26, 0.1)), EXCHANGE(Coordinate(0.25, 0.32)), PASSBRIDGE(Coordinate(0.2, 0.05))
}

class Arm : Controller() {
    private var scope = CoroutineScope(Job())

    val lowerAnglePID = PID(PIDSettings(kP = 0.045, kI = 0.001, kD = 0.0))
    val upperAnglePID = PID(PIDSettings(kP = 0.03, kI = 0.0, kD = 0.001))

    lateinit var lowerMotor: DcMotor
    lateinit var upperMotor: DcMotor

    lateinit var lowerAngle: AnalogInput
    lateinit var upperAngle: AnalogInput

    lateinit var intakeLeft: DcMotor
    lateinit var intakeRight: DcMotor

    private lateinit var clampServo: Servo
    private lateinit var turningServo: Servo


    var isClawTurning = false
    var clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0

    var targetCoordinate = ArmPosition.HOME.coordinate

    var originCoordinate = Coordinate()

    val kinematics = ArmKinematics(armLenght1 = .26, armLength2 = .26)


    var currentState = ArmState.TARGET_LOW
    var lowerTarget = 0.0
    var upperTarget = 0.0

    override fun init(hardwareMap: HardwareMap) {
        lowerMotor = hardwareMap.get(DcMotor::class.java, "lowerMotor")
        upperMotor = hardwareMap.get(DcMotor::class.java, "upperMotor")

        intakeLeft = hardwareMap.get(DcMotor::class.java, "intakeLeft")
        intakeRight = hardwareMap.get(DcMotor::class.java, "intakeRight")

        lowerAngle = hardwareMap.get(AnalogInput::class.java, "lowerAngle")
        upperAngle = hardwareMap.get(AnalogInput::class.java, "upperAngle")


        lowerMotor.direction = DcMotorSimple.Direction.REVERSE
        upperMotor.direction = DcMotorSimple.Direction.REVERSE

        intakeLeft.direction = DcMotorSimple.Direction.REVERSE
        intakeRight.direction = DcMotorSimple.Direction.REVERSE

        clampServo = hardwareMap.get(Servo::class.java, "clampServo")
        turningServo = hardwareMap.get(Servo::class.java, "turningServo")
        clampServo.direction = Servo.Direction.REVERSE
        turningServo.direction = Servo.Direction.REVERSE

    }

    fun moveToCoordinate(x: Double, y: Double) {
        TODO("Implement later for vuforia")
    }

    fun moveToPosition(position: ArmPosition) {
        if (position.coordinate != targetCoordinate) {
            val currentAngles = getAngles()
            originCoordinate = kinematics.calculateFowardKinematics(currentAngles.lowerAngle, currentAngles.upperAngle)
        }

        targetCoordinate = position.coordinate
    }

    fun getAngles(): ArmMotorValues {
        val angles = ArmMotorValues()

        angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0)
        val x = upperAngle.voltage
        angles.upperAngle = -1.66243031965927 * x.pow(3) - 9.090297864880776 * x.pow(2) + 130.359681271249 * x - 137.040643577643

        return angles
    }

    override fun start() {
        turningServo.position = 1.0
    }

    override fun stop() {
        scope.coroutineContext.cancelChildren()
    }


    override fun update() {
        when (currentState) {
            ArmState.TARGET_LOW -> {
                val targetAngles = kinematics.calculateInversedKinematics(targetCoordinate.x, targetCoordinate.y)
                if (targetAngles.lowerAngle + targetAngles.upperAngle > 2) {
                    currentState = ArmState.EXCHANGE
                } else {
                    lowerTarget = (targetAngles.lowerAngle * 180 / PI).coerceIn(55.0, 132.0)
                    upperTarget = (targetAngles.upperAngle * 180 / PI).coerceIn(-135.0, 134.0)
                }
            }
            ArmState.TARGET_HIGH -> {
                val targetAngles = kinematics.calculateInversedKinematics(targetCoordinate.x, targetCoordinate.y)
                if (targetAngles.lowerAngle + targetAngles.upperAngle < 2) {
                    currentState = ArmState.EXCHANGE
                } else {
                    lowerTarget = (targetAngles.lowerAngle * 180 / PI).coerceIn(55.0, 132.0)
                    upperTarget = (targetAngles.upperAngle * 180 / PI).coerceIn(-135.0, 134.0)
                }
            }
            ArmState.EXCHANGE -> {
                val currentAngles = getAngles()
                val targetAngles = kinematics.calculateInversedKinematics(ArmPosition.EXCHANGE.coordinate.x, ArmPosition.EXCHANGE.coordinate.y)
                lowerTarget = targetAngles.lowerAngle * 180 / PI
                upperTarget = targetAngles.upperAngle * 180 / PI

                val currentCoord = kinematics.calculateFowardKinematics(currentAngles.lowerAngle * PI / 180, currentAngles.upperAngle * PI / 180)

                if (currentCoord.closeTo(ArmPosition.EXCHANGE.coordinate, 15.0)) {
                    if (!isClawTurning) {
                        isClawTurning = true
                        turningServo.position = if (turningServo.position == 1.0) 0.0 else 1.0
                        clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0
                    }

                    if (SystemClock.elapsedRealtime() / 1000.0 - clawStartedTurning > 0.5 && isClawTurning) {
                        isClawTurning = false
                        currentState = if (turningServo.position == 0.0) ArmState.TARGET_HIGH else ArmState.TARGET_LOW
                    }
                }
            }
        }

        val currentAngles = getAngles()

        lowerAnglePID.target = lowerTarget
        upperAnglePID.target = upperTarget

        val lowerOutput = lowerAnglePID.update(currentAngles.lowerAngle)
        val upperOutput = upperAnglePID.update(currentAngles.upperAngle)

        lowerMotor.power = lowerOutput
        upperMotor.power = upperOutput
    }


    fun setClampPower(power: Double) {
        val position = 1.0 - power * 0.3
        clampServo.position = position

    }

    fun setServoHeading(degrees: Double) {
        turningServo.position = degrees / 180.0
    }

}

