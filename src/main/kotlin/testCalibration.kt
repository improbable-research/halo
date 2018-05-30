import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

fun main(args : Array<String>) {
    val testParams = HelioStatParameters(
            Vector3D(1.0,1.0,1.0),
            HelioStatParameters.ServoParameters(0.001, 0.1),
            HelioStatParameters.ServoParameters(0.002, 0.2)
    )

    val calibrator = HelioStatCalibration()
//    calibrator.readFromFile("calibrationData.json")
//    calibrator.randomSubSample(10)
    calibrator.createSyntheticTrainingSet(30, testParams)

    calibrator.probabilisticHelioStat.params.pivotPoint.x.value = 1.1
    calibrator.probabilisticHelioStat.params.pivotPoint.y.value = 1.1
    calibrator.probabilisticHelioStat.params.pivotPoint.z.value = 1.1
    calibrator.probabilisticHelioStat.params.cPitch.value = 0.11
    calibrator.probabilisticHelioStat.params.mPitch.value = 0.0011
    calibrator.probabilisticHelioStat.params.cRotation.value = 0.21
    calibrator.probabilisticHelioStat.params.mRotation.value = 0.0021

    val bestParams = calibrator.inferMaxAPosteriori()
    println("Best params are ${bestParams}")
    println("Correct params are: $testParams")
    val r = calibrator.calculateResiduals(bestParams)
    println("average residual is ${r}")
}
