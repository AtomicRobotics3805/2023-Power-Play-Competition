package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam

@Config
@Suppress("Unused", "MemberVisibilityCanBePrivate")
object OpenCVWebcam : Subsystem {

    @JvmField
    var NAME = "Webcam 1"
    @JvmField
    var cameraMonitorViewId: Int = Constants.opMode.hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId",
            "id",
            Constants.opMode.hardwareMap.appContext.getPackageName()
    )

    private lateinit var camera: OpenCvWebcam
    private lateinit var pipeline: PowerPlayPipeline

    var detectedColor = PowerPlayPipeline.SleeveColor.UNDETECTED

    val color: PowerPlayPipeline.SleeveColor
        get() = pipeline.getSleeveColor()

    val detect: DetectCommand
        get() = DetectCommand()

    class DetectCommand : Command(){
        override val _isDone: Boolean
            get() = detectedColor != PowerPlayPipeline.SleeveColor.UNDETECTED

        override fun start(){
            detectedColor = PowerPlayPipeline.SleeveColor.UNDETECTED
        }

        override fun execute(){
            camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
                override fun onOpened() {
                    camera.startStreaming(1920, 1080, OpenCvCameraRotation.SIDEWAYS_LEFT)
                }

                override fun onError(errorCode: Int) {

                }
            })
        }
    }

    override fun initialize(){
        pipeline = PowerPlayPipeline()
        camera = OpenCvCameraFactory.getInstance().createWebcam(Constants.opMode.hardwareMap.get<WebcamName>(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
        camera.setPipeline(pipeline)
    }

}

public class PowerPlayPipeline : OpenCvPipeline() {

    enum class SleeveColor {
        RED,
        GREEN,
        BLUE,
        UNDETECTED
    }

    var greenMat = Mat()
    var blueMat = Mat()
    var redMatNegative = Mat()
    var redMatPositive = Mat()

    var color : SleeveColor = SleeveColor.UNDETECTED

    var redTargetValue = 0.0
    var greenTargetValue = 0.0
    var blueTargetValue = 0.0

    var ROI = Rect(Point(0.0, 1400.0), Point(300.0, 1100.0))

    override fun processFrame(input : Mat): Mat? {
        Imgproc.cvtColor(input, redMatNegative, Imgproc.COLOR_RGB2HSV)
        Imgproc.cvtColor(input, redMatPositive, Imgproc.COLOR_RGB2HSV)
        Imgproc.cvtColor(input, greenMat, Imgproc.COLOR_RGB2HSV)
        Imgproc.cvtColor(input, blueMat, Imgproc.COLOR_RGB2HSV)

        val LowHSVRedNegative = Scalar(245.0, 50.0, 70.0)
        val HighHSVRedNegative = Scalar(255.0, 255.0, 255.0)
        val LowHSVRedPositive = Scalar(0.0, 50.0, 70.0)
        val HighHSVRedPositive = Scalar(20.0, 255.0, 255.0)
        val LowHSVGreen = Scalar(40.0, 50.0, 70.0)
        val HighHSVGreen = Scalar(90.0, 255.0, 255.0)
        val LowHSVBlue = Scalar(100.0, 50.0, 70.0)
        val HighHSVBlue = Scalar(200.0, 255.0, 255.0)

        Core.inRange(redMatPositive, LowHSVRedNegative, HighHSVRedNegative, redMatPositive)
        Core.inRange(redMatNegative, LowHSVRedPositive, HighHSVRedPositive, redMatNegative)
        Core.inRange(greenMat, LowHSVGreen, HighHSVGreen, greenMat)
        Core.inRange(blueMat, LowHSVBlue, HighHSVBlue, blueMat)

        val redTargetPositive: Mat = redMatPositive.submat(ROI)
        val redTargetNegative: Mat = redMatNegative.submat(ROI)
        val greenTarget: Mat = greenMat.submat(ROI)
        val blueTarget: Mat = blueMat.submat(ROI)

        redTargetValue = Core.sumElems(redTargetPositive).`val`[0] / ROI.area() / 255 + Core.sumElems(redTargetNegative).`val`[0] / ROI.area() / 255
        greenTargetValue = Core.sumElems(greenTarget).`val`[0] / ROI.area() / 255
        blueTargetValue = Core.sumElems(blueTarget).`val`[0] / ROI.area() / 255

        redTargetPositive.release()
        redTargetNegative.release()
        greenTarget.release()
        blueTarget.release()

        return greenMat
    }
    public fun getSleeveColor(): SleeveColor{
        color = if (blueTargetValue > redTargetValue && blueTargetValue > greenTargetValue) {
            SleeveColor.BLUE
        } else if (greenTargetValue > redTargetValue) {
            SleeveColor.GREEN
        } else {
            SleeveColor.RED
        }
        return color;
    }
}