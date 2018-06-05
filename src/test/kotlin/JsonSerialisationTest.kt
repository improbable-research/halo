import com.google.gson.Gson
import httpRequests.Json
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.junit.Test

class JsonSerialisationTest {

    @Test
    fun json() {
        val obj = Student("Steve", 9)
        println("Before: $obj")

        val json = Json.toJson(obj)
        println("JSON:")
        println(json)

        val after = Json.fromJson(json, Student::class.java)
        println("After: $after")

        assert(obj.name.equals(after.name))
        assert(obj.age == after.age)
    }

    @Test
    fun jsonKt() {
        val obj = StudentKt("Steve", 9)
        println("Before: $obj")

        val json = Json.toJson(obj)
        println("JSON:")
        println(json)

        val after = Json.fromJson(json, StudentKt::class.java)
        println("After: $after")

        assert(obj.name.equals(after.name))
        assert(obj.age == after.age)
    }

    @Test
    fun serialiseNavigateQuery() {
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

        val gson = Gson()
        val queryJson = gson.toJson(query)

        println(queryJson)

        val rebuiltQuery = Json.fromJson(queryJson, Query.Navigation::class.java)

        assert(query.source.x == rebuiltQuery.source.x)
        assert(query.source.y == rebuiltQuery.source.y)
        assert(query.source.z == rebuiltQuery.source.z)
    }
}
