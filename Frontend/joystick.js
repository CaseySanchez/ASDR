export default {
    name: "Joystick",
    props: {
        width: {
            type: Number,
            default: 100.0,
            coerce: str => Number(str)
        },
        height: {
            type: Number,
            default: 100.0,
            coerce: str => Number(str)
        },
        x_max: {
            type: Number,
            default: 1.0,
            coerce: str => Number(str)
        },
        y_max: {
            type: Number,
            default: 1.0,
            coerce: str => Number(str)
        },
        internal_line_width: {
            type: Number,
            default: 2.0,
            coerce: str => Number(str)
        },
        internal_fill_color: {
            type: String,
            default: "#cccccc"
        },
        internal_stroke_color: {
            type: String,
            default: "#888888"
        },
        external_line_width: {
            type: Number,
            default: 2.0,
            coerce: str => Number(str)
        },
        external_stroke_color: {
            type: String,
            default: "#444444"
        },
        auto_return_to_center: {
            type: Boolean,
            default: true,
            coerce: str => str === "true"
        }
    },
    data() {
        return {
            context: undefined,
            pressed: 0,
            x_position: 0.0,
            y_position: 0.0
        };
    },
    methods: {
        xCenter: function() {
            return this.width * 0.5;
        },
        yCenter: function() {
            return this.height * 0.5;
        },
        xMax: function() {
            return this.width * Math.min(1.0, Math.max(0.0, this.x_max)) * 0.25;
        },
        yMax: function() {
            return this.height * Math.min(1.0, Math.max(0.0, this.y_max)) * 0.25;
        },
        radius: function() {
            return Math.min(this.width * 0.25, this.height * 0.25);
        },
        draw: function() {
            this.context.clearRect(0.0, 0.0, this.width, this.height);
            
            if (Math.abs(this.x_position - this.xCenter()) > this.xMax()) {
                this.x_position = this.xCenter() + this.xMax() * Math.sign(this.x_position - this.xCenter());
            }
    
            if (Math.abs(this.y_position - this.yCenter()) > this.yMax()) {
                this.y_position = this.yCenter() + this.yMax() * Math.sign(this.y_position - this.yCenter());
            }
    
            this.context.beginPath();
    
            this.context.arc(this.xCenter(), this.yCenter(), Math.max(Math.max(this.xMax() * 2.0, this.yMax() * 2.0), 10.0), 0.0, 2.0 * Math.PI, false);
            this.context.rect(this.xCenter() - Math.max(this.xMax() * 2.0, 10.0), this.yCenter() - Math.max(this.yMax() * 2.0, 10.0), Math.max(this.xMax() * 2.0, 10.0) * 2.0, Math.max(this.yMax() * 2.0, 10.0) * 2.0);
    
            this.context.closePath();
    
            this.context.lineWidth = this.external_line_width;
            this.context.strokeStyle = this.external_stroke_color;
    
            this.context.stroke();
    
            this.context.beginPath();

            this.context.arc(this.x_position, this.y_position, this.radius(), 0.0, 2.0 * Math.PI, false);
    
            this.context.closePath();
    
            this.context.fillStyle = this.internal_fill_color;
    
            this.context.fill();
    
            this.context.lineWidth = this.internal_line_width;
            this.context.strokeStyle = this.internal_stroke_color;
    
            this.context.stroke();
        },
        update: function() {    
            this.draw();
    
            const x_value = (this.x_position - this.xCenter()) / this.xMax();
            const y_value = (this.y_position - this.yCenter()) / this.yMax() * -1.0;
    
            this.$emit("joystickMove", x_value, y_value);
        },
        onTouchStart: function(event) {
            this.pressed = 1;
        },    
        onTouchMove: function(event) {
            if (this.pressed === 1 && event.targetTouches[0].target === this.$refs.canvas) {
                this.x_position = event.targetTouches[0].pageX - this.$refs.canvas.offsetLeft;
                this.y_position = event.targetTouches[0].pageY - this.$refs.canvas.offsetTop;
    
                this.update();
            }
        },
        onTouchEnd: function(event) {
            this.pressed = 0;
    
            if (this.auto_return_to_center) {
                this.x_position = this.xCenter();
                this.y_position = this.yCenter();
            }
    
            this.update();
        },    
        onMouseDown: function(event) {
            this.pressed = 1;
        },
        onMouseMove: function(event) {
            if (this.pressed === 1) {
                this.x_position = event.pageX - this.$refs.canvas.offsetLeft;
                this.y_position = event.pageY - this.$refs.canvas.offsetTop;
    
                this.update();
            }
        },    
        onMouseUp: function(event) {
            this.pressed = 0;
    
            if (this.auto_return_to_center) {
                this.x_position = this.xCenter();
                this.y_position = this.yCenter();
            }
    
            this.update();
        }
    },
    setup() {
        var canvas = Vue.ref(null);

        return {
            canvas
        };
    },
    mounted() {
        this.x_position = this.width * 0.5;
        this.y_position = this.height * 0.5;

        this.$refs.canvas.width = this.width;
        this.$refs.canvas.height = this.height;

        this.context = this.$refs.canvas.getContext("2d");

        this.draw();
    },    
    template: `
      <div>
        <canvas ref="canvas" @touchstart="onTouchStart($event)" @touchmove="onTouchMove($event)" @touchend="onTouchEnd($event)" @mousedown="onMouseDown($event)" @mousemove="onMouseMove($event)" @mouseup="onMouseUp($event)">
        </canvas>
      </div>
    `
};