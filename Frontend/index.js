import Joystick from "./joystick.js"

export default {
    name: "index",
    components: {
        Joystick
    },
    data() {
        return {
            state: "Idle",
            loading: false
        };
    },
    methods: {
        onClickState: function() {
            this.loading = true;

            var that = this;

            var query = new URLSearchParams({
                state: this.state
            });

            fetch("http://0.0.0.0:8080/set_state?" + query.toString(), {
                method: "POST",
                headers: {
                    "Content-Type": "text/plain; charset=UTF-8",
                    "Access-Control-Allow-Origin": "*"
                }
            })
            .then(function(data) {
                that.loading = false;

                console.log("SUCCESS: ", data);
            })
            .catch(function(error) {
                that.loading = false;

                console.log("FAILURE: ", error);
            });            
        },
        onJoystickMoveRotate: function(x_value, y_value) {
        },
        onJoystickMoveTranslate: function(x_value, y_value) {
        }
    },
    created() {
    },
    template: `
        <v-app id="app">
            <v-container>
                <v-row>
                    <v-col>
                    </v-col>
                    <v-col cols="10">
                        <v-row align-center justify-center>
                            <v-col>
                                <h1>Autonomous Surface Disinfection Robot</h1>
                            </v-col>
                        </v-row>
                        <v-row align-center justify-center class="my-16">
                            <v-col>
                                <v-btn-toggle tile group color="deep-purple accent-3" v-model="state">
                                    <v-btn value="Idle" @click="onClickState()">Idle</v-btn>
                                    <v-btn value="Manual" @click="onClickState()">Manual</v-btn>
                                    <v-btn value="Automatic" @click="onClickState()">Automatic</v-btn>
                                </v-btn-toggle>
                            </v-col>
                        </v-row>
                        <v-row align-center justify-center class="my-16">
                            <v-col>
                                <div v-if="loading === true">
                                    <v-row>
                                        <v-col>
                                            <v-progress-circular color="deep-purple" :size="50" :width="5" indeterminate></v-progress-circular>
                                        </v-col>
                                    </v-row>
                                </div>
                                <div v-else-if="state === 'Idle'">
                                </div>
                                <div v-else-if="state === 'Manual'">
                                    <v-row>
                                        <v-col>
                                            <Joystick ref="joystick_rotate" :width="200" :height="200" :x_max="1.0" :y_max="0.0" @joystickMove="onJoystickMoveRotate"></Joystick>
                                        </v-col>
                                        <v-col>
                                            <Joystick ref="joystick_translate" :width="200" :height="200" :x_max="0.0" :y_max="1.0" @joystickMove="onJoystickMoveTranslate"></Joystick>
                                        </v-col>
                                    </v-row>
                                </div>
                                <div v-else-if="state === 'Automatic'">
                                </div>
                            </v-col>
                        </v-row>
                    </v-col>
                    <v-col>
                    </v-col>
                </v-row>
            </v-container>
        </v-app>
        `
};