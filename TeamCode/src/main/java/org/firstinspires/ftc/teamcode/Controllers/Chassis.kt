package org.firstinspires.ftc.teamcode.Controllers

import android.os.SystemClock
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.ReceiveChannel
import kotlinx.coroutines.channels.SendChannel
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.core.Twist2D
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import kotlin.math.withSign

data class MecanumMotorValues(var topLeftSpeed: Double = 0.0, var topRightSpeed: Double = 0.0, var downLeftSpeed: Double = 0.0, var downRightSpeed: Double = 0.0){
    operator fun plus(other : MecanumMotorValues) : MecanumMotorValues {
        val result = MecanumMotorValues()
        result.topLeftSpeed = this.topLeftSpeed + other.topLeftSpeed
        result.topRightSpeed = this.topRightSpeed + other.topRightSpeed
        result.downLeftSpeed = this.downLeftSpeed + other.downLeftSpeed
        result.downRightSpeed = this.downRightSpeed + other.downRightSpeed
        return result
    }
    operator fun minus(other : MecanumMotorValues) : MecanumMotorValues {
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


class Chassis : Controller() {
    private val scope = CoroutineScope(Job())
    private lateinit var topLeftMotor: DcMotorEx
    private lateinit var topRightMotor: DcMotorEx
    private lateinit var downRightMotor: DcMotorEx
    private lateinit var downLeftMotor: DcMotorEx

    private lateinit var angularPID: PID
    private lateinit var pidSettingsNormal : PIDSettings
    private lateinit var pidSettingsVy : PIDSettings

    private lateinit var imu: BNO055IMU

    private var kinematics = MecanumKinematics(0.5, 0.5, 1.0)

    var movementTarget = Twist2D()
        set(value) {
            field = value
            angularPID.target = field.w
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

        pidSettingsNormal = PIDSettings(kP = 0.032, kI = 0.0, kD = 0.0018, continous = true, lowerBound = -180.0, upperBound = 180.0)
        pidSettingsVy = PIDSettings(kP = 0.08, kI = 0.0, kD = 0.0, continous = true, lowerBound = -180.0, upperBound = 180.0)

        angularPID = PID(pidSettingsNormal)

    }

    @ExperimentalCoroutinesApi
    override fun start() {
        scope.launch { headingProducer(angularPID.inputChannel) }
        scope.launch { angularPID.start() }
        scope.launch { motorVelocityReceiver(angularPID.outputChannel) }
    }

    override fun stop() {
        scope.cancel()
    }

    @ExperimentalCoroutinesApi
    suspend fun headingProducer(sendChannel: SendChannel<Double>) {
        while (scope.isActive) {
            sendChannel.send(getHeading())
        }
    }


    suspend fun motorVelocityReceiver(angularVelocityChannel: ReceiveChannel<Double>) {
        while (scope.isActive) {
            if(Math.abs(movementTarget.vy) > Math.abs(movementTarget.vx)){
                angularPID.pidSettings = pidSettingsVy
            }else{
                angularPID.pidSettings = pidSettingsNormal
            }
            val motorValues = kinematics.calcInverseKinematics(movementTarget.vx, movementTarget.vy, angularVelocityChannel.receive())
            writeMotors(motorValues)
        }

    }


    fun getHeading(): Double {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
    }
//
//    fun moveWithoutPID(vx: Double, vy: Double, w: Double) {
//        val values = kinematics.calcInverseKinematics(vx, vy, w)
//        writeMotors(values)
//    }

    suspend fun writeMotors(values: MecanumMotorValues) {
        topLeftMotor.power = values.topLeftSpeed
        topRightMotor.power = values.topRightSpeed
        downLeftMotor.power = values.downLeftSpeed
        downRightMotor.power = values.downRightSpeed
    }
}
