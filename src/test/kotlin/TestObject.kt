class TestObject() {

    lateinit var name: String
    var number: Int = 0

    constructor(name: String, number: Int): this() {
        this.name = name
        this.number = number
    }

    override fun toString(): String {
        return "[name = $name, number = $number]"
    }
}
