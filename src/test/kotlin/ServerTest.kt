import httpRequests.HttpRequest
import httpRequests.JsonUtils
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
    fun json() {
        val obj = Student("Steve", 9)
        println("Before: $obj")

        val json = JsonUtils.toJson(obj)
        println("JSON:")
        println(json)

        val after = JsonUtils.jsonToClass(json, Student::class.java)
        println("After: $after")
    }

    @Test
    fun jsonKt() {
        val obj = TestObject("Steve", 9)
        println("Before: $obj")

        val json = JsonUtils.toJson(obj)
        println("JSON:")
        println(json)

        val after = JsonUtils.jsonToClass(json, TestObject::class.java)
        println("After: $after")
    }

    private fun startServer() {
        val server = Server()
        server.start()
    }

//    private class TestObject(val name: String, val number: Int) {
//
//        override fun toString(): String {
//            return "[name = $name, number = $number]"
//        }
//    }
}
