import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.junit.Test

class ServerTest {

    @Test
    fun testPing() {
        Server().start()
        ping()
    }

    @Test
    fun testSetMirrorNormal() {
        Server().start()
        setMirrorNormal()
    }

    @Test
    fun testNavigate() {
        Server().start()
        navigate()
    }

    @Test
    fun testNavigatePointToPoint() {
        Server().start()
        navigatePointToPoint()
    }

    @Test
    fun testCalibrateFromSyntheticData() {
        Server().start()
        calibrateFromSyntheticData()
    }

    @Test
    fun testAll() {
        Server().start()
        ping()
        setMirrorNormal()
        navigate()
        navigatePointToPoint()
        calibrateFromSyntheticData()
    }

    private fun ping() {
        val responseGet = Http.get("http://localhost:8080/ping", "")
        println("Response (get): $responseGet")

        val responsePost = Http.post("http://localhost:8080/ping", "")
        println("Response (post): $responsePost")
    }

    private fun setMirrorNormal() {
        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1, -0.02 + Math.PI / 2.0, -Math.PI/2.0),
                HelioStatParameters.ServoParameters(0.002, 0.2, Math.PI - 0.02, 0.0)
        )

        val model = HelioStat(testParams)
        val pitch = 1000
        val rotation = 345
        val normal = model.computeHeliostatNormal(ServoSetting(rotation, pitch)).getValue()

        val query = Query.SetNormal(testParams, normal)
        val jsonQuery = Json.toJson(query)

        val response = Http.post("http://localhost:8080/setMirrorNormal", jsonQuery)

        val servoSetting = Json.fromJson(response, ServoSetting::class.java)

        println("Setting rotation/pitch is ${servoSetting.rotation} ${servoSetting.pitch}")
        assert(servoSetting.rotation == rotation)
        assert(servoSetting.pitch == pitch)
    }

    private fun navigate() {
        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1, 0.015 + Math.PI / 2.0, -Math.PI / 2),
                HelioStatParameters.ServoParameters(0.002, 0.2, Math.PI - 0.03, Math.PI * 0.7)
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

        val queryJson = Json.toJson(query)

        val response = Http.post("http://localhost:8080/navigate", queryJson)

        val servoSetting = Json.fromJson(response, ServoSetting::class.java)
        println("Servo setting pitch/rotation is ${servoSetting.pitch} ${servoSetting.rotation}")
        assert(servoSetting.pitch == pitch)
        assert(servoSetting.rotation == rotation)
    }

    private fun navigatePointToPoint() {
        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1, 0.01 + Math.PI / 2.0, -Math.PI / 2),
                HelioStatParameters.ServoParameters(0.002, 0.2, -Math.PI - 0.03, -Math.PI)
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

        val queryJson = Json.toJson(query)

        val response = Http.post("http://localhost:8080/navigatePointToPoint", queryJson)

        val servoSetting = Json.fromJson(response, ServoSetting::class.java)
        println("Servo setting pitch/rotation is ${servoSetting.pitch} ${servoSetting.rotation}")
        assert(servoSetting.pitch == pitch)
        assert(servoSetting.rotation == rotation)
    }

    private fun calibrateFromSyntheticData() {
        // TODO: Need to either calibrate in cartesian coords or add atan2 vertex for successful conversion
        // TODO: from cartesian to spherical coords

        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1, Math.PI / 2.0, -Math.PI / 2.0),
                HelioStatParameters.ServoParameters(0.002, 0.2, Math.PI, 0.0)
        )

        val calibrationData = CalibrationData()
        calibrationData.createSyntheticTrainingSet(40, testParams)

        val calibrationDataJson = Json.toJson(calibrationData)
        val response = Http.post("http://localhost:8080/calibrateInternalDataFormat", calibrationDataJson)

        val calibrator = HelioStatCalibrator(calibrationData)

        val bestParams: HelioStatParameters = Json.fromJson(response, HelioStatParameters::class.java)

        println("Modelled params are: $bestParams")
        val r = calibrator.calculateResiduals(bestParams)
        val residual = r.sumByDouble(Vector3D::getNorm) / r.size
        println("average residual is $residual")
        assert(residual < 1e-2)
    }
}
