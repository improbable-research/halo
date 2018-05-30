import io.javalin.ApiBuilder.get
import io.javalin.ApiBuilder.post
import io.javalin.Javalin

class Server {
    val app = Javalin.create().apply {
        port(8080)
        exception(Exception::class.java) {e, ctx -> e.printStackTrace()}
        error(404) {ctx -> ctx.json("not found")}
    }

    constructor() {
        app.routes {

            post("/postTest") {ctx ->
                ctx.json(TestPayload("got post test", 1234))
            }

            post("/payloadTest") {ctx ->
                val payload = ctx.bodyAsClass(TestPayload::class.java)
                ctx.json(payload)
            }

            get("/getTest/:id") { ctx ->
                ctx.json(TestPayload(ctx.queryParam("x")?:"no value", ctx.param("id").orEmpty().toInt()))
            }

            get("/test") { ctx ->
                ctx.json(HelioStatCalibration(listOf(HelioStatCalibration.DataPoint(listOf(0.1,0.2,0.3), listOf(1,2)))))
            }


            post("/runModel") { ctx ->

                //response in JSON
            }

            post("/navigateToPoint") { ctx ->
                // get params, present control, bounds, point, solar vector
                // return new control params
            }

            post("/navigatePointToPoint") { ctx ->
                // get params, present control, bounds, point, second point
                // return new control params

            }

            post("/calibrate") { ctx ->
                val payload = ctx.bodyAsClass(TestPayload::class.java)
                ctx.status(201)

                // receive whole caboodle in JSON format
                // return parameters
            }

        }
    }

    fun start() : Javalin {
        return app.start()
    }

}