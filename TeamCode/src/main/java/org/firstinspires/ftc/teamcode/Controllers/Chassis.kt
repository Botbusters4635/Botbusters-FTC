package org.firstinspires.ftc.teamcode.Controllers

import android.os.SystemClock
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.core.Twist2D
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import kotlin.math.PI

data class MecanumMotorValues(var topLeftSpeed: Double = 0.0, var topRightSpeed: Double = 0.0, var downLeftSpeed: Double = 0.0, var downRightSpeed: Double = 0.0) {
    operator fun plus(other: MecanumMotorValues): MecanumMotorValues {
        val result = MecanumMotorValues()
        result.topLeftSpeed = this.topLeftSpeed + other.topLeftSpeed
        result.topRightSpeed = this.topRightSpeed + other.topRightSpeed
        result.downLeftSpeed = this.downLeftSpeed + other.downLeftSpeed
        result.downRightSpeed = this.downRightSpeed + other.downRightSpeed
        return result
    }

    operator fun minus(other: MecanumMotorValues): MecanumMotorValues {
        val result = MecanumMotorValues()
        result.topLeftSpeed = this.topLeftSpeed - other.topLeftSpeed
        result.topRightSpeed = this.topRightSpeed - other.topRightSpeed
        result.downLeftSpeed = this.downLeftSpeed - other.downLeftSpeed
        result.downRightSpeed = this.downRightSpeed - other.downRightSpeed
        return result
    }
}


class MecanumKinematics(var xDistanceFromWheelToCenter: Double, var yDistanceFromWheelToCenter: Double, var wheelRadius: Double) {

    fun calcInverseKinematics(vx: Double, vy: Double, w: Double): MecanumMotorValues {
        val values = MecanumMotorValues()
        values.topLeftSpeed = (vx - vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.topRightSpeed = (vx + vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.downLeftSpeed = (vx + vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.downRightSpeed = (vx - vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        return values

    }


    fun calcForwardKinematics(state: MecanumMotorValues): Twist2D {
        val result = Twist2D()

        result.vx = (state.topLeftSpeed + state.topRightSpeed + state.downLeftSpeed + state.downRightSpeed) * wheelRadius / 4.0
        result.vy = (-state.topLeftSpeed + state.topRightSpeed + state.downLeftSpeed - state.downRightSpeed) * wheelRadius / 4.0
        result.w = (-state.topLeftSpeed + state.topRightSpeed - state.downLeftSpeed + state.downRightSpeed) * wheelRadius / 4.0 * (xDistanceFromWheelToCenter + yDistanceFromWheelToCenter)
        return result
    }
}

enum class ChassisMode {
    PID, OPEN
}


class Chassis : Controller() {
    private val scope = CoroutineScope(Job())

    private val pidSettingsNormal = PIDSettings(kP = 0.04, kI = 0.0, kD = 0.00, continous = true, lowerBound = -180.0, upperBound = 180.0)

    private val angularPID = PID(pidSettingsNormal, scope)

    private lateinit var topLeftMotor: DcMotorEx
    private lateinit var topRightMotor: DcMotorEx
    private lateinit var downRightMotor: DcMotorEx
    private lateinit var downLeftMotor: DcMotorEx

    private lateinit var imu: BNO055IMU

    private var currentMode: ChassisMode = ChassisMode.OPEN

    private var kinematics = MecanumKinematics(0.5, 0.5, 1.0)

    var lastTimeRun = SystemClock.elapsedRealtime() / 1000.0

    var movementTarget = Twist2D()
        set(value) {
            val timeStep = (SystemClock.elapsedRealtime() / 1000.0) - lastTimeRun
            field = value

            var target = angularPID.target + field.w * timeStep

            if (target > 180) {
                target -= 360
            }
            if (target < -180) {
                target += 360
            }

            angularPID.target = target
            lastTimeRun = SystemClock.elapsedRealtime() / 1000.0
        }


    override fun init(hardwareMap: HardwareMap) {
        // Get motors and cast to DcMotorEx
        topLeftMotor = hardwareMap.get(DcMotor::class.java, "topLeftMotor") as DcMotorEx
        topRightMotor = hardwareMap.get(DcMotor::class.java, "topRightMotor") as DcMotorEx
        downLeftMotor = hardwareMap.get(DcMotor::class.java, "downLeftMotor") as DcMotorEx
        downRightMotor = hardwareMap.get(DcMotor::class.java, "downRightMotor") as DcMotorEx


        topLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        topRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        downRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        downLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Reverse motors
        topLeftMotor.direction = DcMotorSimple.Direction.REVERSE
        downLeftMotor.direction = DcMotorSimple.Direction.REVERSE

        // Get the IMU, configure it and initialize it.
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.mode = BNO055IMU.SensorMode.NDOF
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        imu.initialize(parameters)

        // Initialize the angularPID

    }

    @ExperimentalCoroutinesApi
    override fun start() {
        headingProducer()
        angularPID.start()
        headingReceiver()
    }

    override fun stop() {
        scope.coroutineContext.cancelChildren()
    }

    fun headingProducer() = scope.launch {
        while (isActive) {
            angularPID.inputChannel.send(getHeading())
        }
    }

    fun headingReceiver() = scope.launch {
        while (isActive) {
            var motorValues: MecanumMotorValues
            telemetry.addData("currentMode", currentMode)
            when (currentMode) {
                ChassisMode.OPEN -> {
                    if (movementTarget.vy != 0.0) {
                        currentMode = ChassisMode.PID
                        angularPID.target = getHeading()
                        motorValues = MecanumMotorValues(0.0, 0.0, 0.0)
                        writeMotors(motorValues)
                        delay(10)
                    } else {
                        motorValues = kinematics.calcInverseKinematics(movementTarget.vx, 0.0, movementTarget.w)
                        writeMotors(motorValues)

                    }

                }
                ChassisMode.PID -> {
                    if(movementTarget.vy == 0.0){
                        currentMode = ChassisMode.OPEN
                    } else {
                        motorValues = kinematics.calcInverseKinematics(0.0, movementTarget.vy, angularPID.outputChannel.receive())
                        writeMotors(motorValues)
                    }
                }
            }

        }
    }


    fun getHeading(): Double {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
    }

    fun writeMotors(values: MecanumMotorValues) {
        topLeftMotor.power = values.topLeftSpeed
        topRightMotor.power = values.topRightSpeed
        downLeftMotor.power = values.downLeftSpeed
        downRightMotor.power = values.downRightSpeed
    }
}
