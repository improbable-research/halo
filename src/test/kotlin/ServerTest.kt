import httpRequests.HttpRequest
import httpRequests.JsonUtils
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.junit.Test

class ServerTest {

    @Test
    fun ping() {
        startServer()
        val responseGet = HttpRequest.get("http://localhost:8080/ping", "")
        println("Response (get): $responseGet")

        val responsePost = HttpRequest.post("http://localhost:8080/ping", "")
        println("Response (post): $responsePost")
    }

    @Test
    fun testNavigate() {
        startServer()

        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1, 0.015 + Math.PI / 2.0, -Math.PI/2 + 0.01),
                HelioStatParameters.ServoParameters(0.002, 0.2, Math.PI-0.03, Math.PI * 0.7)
        )

        val model = HelioStat(testParams)
        val pitch = 1234
        val rotation = 2345

        val sunVector = Vector3D(1.0, -0.1, 1.0).normalize()
        val distance = 4.0
        val correctServoSetting = ServoSetting(rotation, pitch)
        val target = model.computeTargetFromSourceDirection(
                ConstantDoubleVertex(correctServoSetting.pitch.toDouble()),
                ConstantDoubleVertex(correctServoSetting.rotation.toDouble()),
                ProbabilisticVector3D(sunVector),
                ConstantDoubleVertex(distance)).getValue()

        val currentServoSetting = ServoSetting(rotation + 100, pitch + 100)

        val query = Query.Navigation(testParams, currentServoSetting, target, sunVector)

        val queryJson = JsonUtils.toJson(query)

        val response = HttpRequest.post("http://localhost:8080/navigate", queryJson)

        val servoSetting = JsonUtils.fromJson(response.toString(), ServoSetting::class.java)
        println("Servo setting pitch/rotation is ${servoSetting.pitch} ${servoSetting.rotation}")
        assert(servoSetting.pitch == pitch)
        assert(servoSetting.rotation == rotation)
    }

    @Test
    fun testNavigatePointToPoint() {
        startServer()

        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1, 0.01 + Math.PI / 2.0, -Math.PI/2 + 0.01),
                HelioStatParameters.ServoParameters(0.002, 0.2, -Math.PI-0.03, -Math.PI)
        )

        val navigator = HelioStatNavigator(testParams)
        val model = HelioStat(testParams)
        val pitch = 1000
        val rotation = 345


        val sourcePoint = Vector3D(1.0, 2.0, 10.0)
        val distance = 5.5
        val correctServoSetting = ServoSetting(rotation, pitch)
        val target = model.computeTargetFromSourcePoint(
                ConstantDoubleVertex(correctServoSetting.pitch.toDouble()),
                ConstantDoubleVertex(correctServoSetting.rotation.toDouble()),
                sourcePoint,
                ConstantDoubleVertex(distance)
        ).getValue()

        val currentServoSetting = navigator.computeServoSettingFromPoint(sourcePoint, target,
                ServoSetting(rotation + 100, pitch + 100)
        )

        val query = Query.Navigation(testParams, currentServoSetting, target, sourcePoint)

        val queryJson = JsonUtils.toJson(query)

        val response = HttpRequest.post("http://localhost:8080/navigatePointToPoint", queryJson)

        val servoSetting = JsonUtils.fromJson(response.toString(), ServoSetting::class.java)
        println("Servo setting pitch/rotation is ${servoSetting.pitch} ${servoSetting.rotation}")
        assert(servoSetting.pitch == pitch)
        assert(servoSetting.rotation == rotation)
    }

    private fun startServer() {
        val server = Server()
        server.start()
    }
}
