import io.javalin.ApiBuilder.get
import io.javalin.ApiBuilder.post
import io.javalin.Javalin


fun main(args : Array<String>) {

    val server = Server()

    server.start()

//    var response = mutableMapOf("field1" to TestPayload("hello", 1234))
//
//    val app = Javalin.create().apply {
//        port(8080)
//        exception(Exception::class.java) {e, ctx -> e.printStackTrace()}
//        error(404) {ctx -> ctx.json("not found")}
//    }.start()
//
//    app.routes {
//        get("/test/:id") { ctx ->
//            response["field1"]?.id = ctx.param("id").orEmpty().toInt()
//            response["field1"]?.name = ctx.queryParam("x")?:"no value"
//            ctx.json(response)
//
//        }
//
//        post("/runModel") { ctx ->
//
//            //response in JSON
//        }
//
//        post("navigate") { ctx ->
//            // get params, present control, bounds, point, solar vector
//            // return new control params
//        }
//
//        post("navigatePointToPoint") {ctx ->
//            // get params, present control, bounds, point, second point
//            // return new control params
//
//        }
//
//        post("/calibrate") { ctx ->
//            val payload = ctx.bodyAsClass(TestPayload::class.java)
//            ctx.status(201)
//
//            // receive whole caboodle in JSON format
//            // return parameters
//        }
//
//    }
}
