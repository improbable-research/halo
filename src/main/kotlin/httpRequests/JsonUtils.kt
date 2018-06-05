package httpRequests

import com.fasterxml.jackson.databind.ObjectMapper

class JsonUtils {

    companion object {

        private val mapper = ObjectMapper()

        fun <T> toJson(obj: T): String {
            return mapper.writerWithDefaultPrettyPrinter().writeValueAsString(obj)
        }

        fun <T> jsonToClass(json: String, type: Class<T>): T {
            return mapper.readValue(json, type)
        }
    }

}
