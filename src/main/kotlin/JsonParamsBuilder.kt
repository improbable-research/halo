import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import java.io.File

fun main(args: Array<String>) {

    val skipLines = 1
    val reader = File("file.csv").inputStream().bufferedReader(Charsets.UTF_8)
    val lines = reader.readLines()
    lines.subList(skipLines, lines.size)

    val hamParameters = arrayListOf<HamParameters>()
    for (line in lines) {
        val lineFields = line.split(",")
        hamParameters.add(csvToHamParametersJson(lineFields))
    }

    Json.toJson(hamParameters)
}

private fun csvToHamParametersJson(lineFields: List<String>): HamParameters {
    val id = lineFields[0].toInt()
    val pivotX = lineFields[1].toDouble()
    val pivotY = lineFields[2].toDouble()
    val pivotZ = lineFields[3].toDouble()
    val targetX = lineFields[4]
    val targetY = lineFields[5]
    val targetZ = lineFields[6]
    val rotationXAxisSetting = lineFields[7]
    val pitchYAxisSetting = lineFields[8]

    val pivotPoint = Vector3D(pivotX, pivotY, pivotZ)
    val helioParams = HelioStatParameters(pivotPoint)
    val axis1servoID = id * 2 - 1
    val axis2servoID = id * 2
    return HamParameters(id, axis1servoID, axis2servoID, helioParams)
}



