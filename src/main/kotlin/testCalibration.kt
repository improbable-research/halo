
fun main(args : Array<String>) {
    val calibrator = HelioStatCalibration()
    calibrator.readFromFile("calibrationData.json")
    val bestParams = calibrator.inferMaxAPosteriori()
    println("Best params are ${bestParams}")
    val r = calibrator.calculateResiduals(bestParams)
    println("average residual is ${r}")
}
