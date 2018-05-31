import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

fun main(args : Array<String>) {
    val testParams = HelioStatParameters(
            Vector3D(1.0,1.0,1.0),
            HelioStatParameters.ServoParameters(0.001, 0.1),
            HelioStatParameters.ServoParameters(0.002, 0.2)
    )

    val dataReader = CalibrationDataReadAndConvert()
    dataReader.readFromFile("calibrationData.json")
    dataReader.randomSubSample(20)

    val calibrator = HelioStatCalibration(dataReader)

//    calibrator.createSyntheticTrainingSet(30, testParams)

//    calibrator.probabilisticHelioStat.params.pivotPoint.x.value = 100.1
//    calibrator.probabilisticHelioStat.params.pivotPoint.y.value = 100.2
//    calibrator.probabilisticHelioStat.params.pivotPoint.z.value = 200.3
//    calibrator.probabilisticHelioStat.params.cPitch.value = 0.11
//    calibrator.probabilisticHelioStat.params.mPitch.value = 0.0011
//    calibrator.probabilisticHelioStat.params.cRotation.value = 0.21
//    calibrator.probabilisticHelioStat.params.mRotation.value = 0.0021

//    val pivot = calibrator.inferPivotPoint()
//    println("Pivot is: $pivot")
//
//    val servoParams = calibrator.inferServoParams()
//    println("Servo Params are: $servoParams")

    val bestParams = calibrator.inferAllParams()
//    println("Correct params are: $testParams")
    println("Modelled params are: $bestParams")
    val r = calibrator.calculateResiduals(bestParams)
    val residual = r.sumByDouble(Vector3D::getNorm)/r.size
    println("average residual is $residual")

    for(i in 0 until calibrator.size) {
        println("${calibrator[i].control.pitch} ${calibrator[i].control.rotation} ${r[i].x} ${r[i].y} ${r[i].z}")
    }
}
