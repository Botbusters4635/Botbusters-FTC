package org.firstinspires.ftc.teamcode.controllers

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import kotlin.math.*

data class ArmAngleValues(var lowerAngle: Double = 0.00, var upperAngle: Double = 0.00)


class ArmKinematics(val armLenght1: Double, val armLength2: Double) {


    fun calculateInverseKinematics(targetCoord: Coordinate): ArmAngleValues {
        val values = ArmAngleValues()
        var cosAngle = (targetCoord.x.pow(2) + targetCoord.y.pow(2) - armLenght1.pow(2) - armLength2.pow(2)) / (2 * armLenght1 * armLength2)

        if (cosAngle > 1) {
            cosAngle = 1.0
        } else if (cosAngle < -1.0) {
            cosAngle = -1.0
        }


        if (targetCoord.x > 0) {
            values.upperAngle = -acos(cosAngle)
            values.lowerAngle = atan2(targetCoord.y, targetCoord.x) + atan2((armLength2 * sin(abs(values.upperAngle))), (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        } else {
            values.upperAngle = acos(cosAngle)
            values.lowerAngle = atan2(targetCoord.y, targetCoord.x) - atan2((armLength2 * sin(abs(values.upperAngle))), (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        }
        return values

    }

    fun calculateFowardKinematics(armValues: ArmAngleValues): Coordinate {
        val values2 = Coordinate()
        values2.x = cos(armValues.lowerAngle) * armLenght1 + cos(armValues.lowerAngle + armValues.upperAngle) * armLength2
        values2.y = sin(armValues.lowerAngle) * armLenght1 + sin(armValues.lowerAngle + armValues.upperAngle) * armLength2

        return values2
    }


}

enum class ArmState {
    GO_TARGET, EXCHANGE_BACK_TO_FRONT, EXCHANGE_FRONT_TO_BACK
}

enum class TURN_POS(val value: Double) {
    OPEN(0.0), CLOSED(1.0), MIDDLE(0.5)
}

enum class ArmPosition(val coordinate: Coordinate) {
    SLOW(Coordinate(-0.42, 0.2)), HOME(Coordinate(0.20, 0.08)), TOP(Coordinate(-0.26, 0.3)), MEDIUM(Coordinate(-0.26, 0.15)), LOW(Coordinate(-0.26, 0.05)), EXCHANGE(Coordinate(0.25, 0.32)),INTAKE(Coordinate(0.25, 0.28)),  PASSBRIDGE(Coordinate(0.2, 0.05))
}

class Arm : Controller() {
    private var scope = CoroutineScope(Job())

    val lowerAnglePID = PID(PIDSettings(kP = 0.045, kI = 0.001, kD = 0.0))
    val upperAnglePID = PID(PIDSettings(kP = 0.02, kI = 0.0, kD = 0.00015))

    lateinit var lowerMotor: DcMotor
    lateinit var upperMotor: DcMotor

    lateinit var lowerAngle: AnalogInput
    lateinit var upperAngle: AnalogInput

    lateinit var intakeLeft: DcMotor
    lateinit var intakeRight: DcMotor

    private lateinit var clampServo: Servo
    private lateinit var turningServo: Servo


    var clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0
    var clawTurnTime = 0.5 //Seconds needed for claw to turn to position

    var targetCoordinate = ArmPosition.HOME.coordinate

    var originCoordinate = Coordinate()

    val kinematics = ArmKinematics(armLenght1 = .26, armLength2 = .26)

    var currentState = ArmState.GO_TARGET

    var clawTurning = false

    var servoPosition = 0.0


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
        setClampPower(0.0)
    }

    fun moveToCoordinate(x: Double, y: Double) {
        TODO("Implement later for vuforia")
    }

    fun moveToPosition(position: ArmPosition) {
        val currentAngles = getAngles()
        val currentCoordinate = kinematics.calculateFowardKinematics(currentAngles)


        if (position.coordinate != targetCoordinate) {
            if(currentCoordinate.x > 0.0 && position.coordinate.x < 0.0){
                currentState = ArmState.EXCHANGE_FRONT_TO_BACK
            }else if(currentCoordinate.x < 0.0 && position.coordinate.x > 0.0){
                currentState = ArmState.EXCHANGE_BACK_TO_FRONT
            }else{
                currentState = ArmState.GO_TARGET
            }
        }
        targetCoordinate = position.coordinate

    }

    fun getAngles(radians: Boolean = true): ArmAngleValues {
        val angles = ArmAngleValues()

        angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0)
        val x = upperAngle.voltage
        angles.upperAngle = -1.66243031965927 * x.pow(3) - 9.090297864880776 * x.pow(2) + 130.359681271249 * x - 137.040643577643

        if(radians){
            angles.lowerAngle *= Math.PI / 180.0
            angles.upperAngle *= Math.PI / 180.0
        }

        return angles
    }

    override fun start() {
        turningServo.position = TURN_POS.CLOSED.value
    }

    override fun stop() {
        scope.coroutineContext.cancelChildren()
    }


    override fun update(timeStep: Double) {
        val currentAngles = getAngles()
        val currentCoordinate = kinematics.calculateFowardKinematics(currentAngles)

        var currentTargetCoord = targetCoordinate
        when (currentState) {
            ArmState.EXCHANGE_FRONT_TO_BACK -> {
                if(currentCoordinate.closeTo(ArmPosition.EXCHANGE.coordinate)){
                    setServoHeading(180.0)
                    if(!clawTurning){
                        clawTurning = true
                        clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0
                    }else if(SystemClock.elapsedRealtime() / 1000.0 - clawStartedTurning > clawTurnTime){
                        clawTurning = false
                        servoPosition = 180.0
                        currentState = ArmState.GO_TARGET
                    }
                }
                currentTargetCoord = ArmPosition.EXCHANGE.coordinate
            }
            ArmState.EXCHANGE_BACK_TO_FRONT -> {
                if(currentCoordinate.closeTo(ArmPosition.EXCHANGE.coordinate)){
                    setServoHeading(0.0)
                    if(!clawTurning){
                        clawTurning = true
                        clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0
                    }else if(SystemClock.elapsedRealtime() / 1000.0 - clawStartedTurning > clawTurnTime){
                        clawTurning = false
                        servoPosition = 0.0
                        currentState = ArmState.GO_TARGET
                    }
                }
                currentTargetCoord = ArmPosition.EXCHANGE.coordinate
            }
            ArmState.GO_TARGET -> {
                currentTargetCoord = targetCoordinate
            }
        }

        telemetry.addData("targetCoord", currentTargetCoord)
        telemetry.addData("currentCoord", "%.2f %.2f", currentCoordinate.x, currentCoordinate.y)
        telemetry.addData("Angles", getAngles())
        telemetry.addData("state", currentState)

        val targetAngles = kinematics.calculateInverseKinematics(currentTargetCoord)

        lowerAnglePID.target = targetAngles.lowerAngle
        upperAnglePID.target = targetAngles.upperAngle

        val lowerOutput = lowerAnglePID.update(currentAngles.lowerAngle * 180.0 / Math.PI, timeStep)
        val upperOutput = upperAnglePID.update(currentAngles.upperAngle  * 180.0 / Math.PI, timeStep)

        lowerMotor.power = lowerOutput
        upperMotor.power = upperOutput
    }

    fun setClampPower(power: Double) {
        val position = 1.0 - power * 0.2
        clampServo.position = position

    }

    fun setServoHeading(degrees: Double) {
        turningServo.position = (180.0- degrees) / 180.0
    }

    fun runToPositionCommand(position: ArmPosition) = runBlocking {
        moveToPosition(position)
        var currentAngles = getAngles()
        var currentCoordinate = kinematics.calculateFowardKinematics(currentAngles)

        while (!currentCoordinate.closeTo(position.coordinate)) {
            currentAngles = getAngles()
            currentCoordinate = kinematics.calculateFowardKinematics(currentAngles)
        }
    }
}

