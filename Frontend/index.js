import Joystick from "./joystick.js"

export default {
    name: "index",
    components: {
        Joystick
    },
    data() {
        return {
            error: undefined,
            state: undefined,
            loading: false,
            x_value: 0.0,
            y_value: 0.0
        };
    },
    watch: {
        state: function(new_value, old_value) {
            if (new_value !== undefined && old_value !== undefined) {
                var self = this;

                async function set_state() {
                    self.loading = true;

                    var query = new URLSearchParams({
                        state: new_value
                    });

                    await fetch("http://0.0.0.0:8080/set_state?" + query.toString(), {
                        method: "POST",
                        headers: {
                            "Content-Type": "text/plain; charset=UTF-8"
                        }
                    })
                    .then(function(response) {
                        console.log("SUCCESS: " + response);
                        
                        self.loading = false;
                    })
                    .catch(function(response) {
                        console.log("FAILURE: " + response);

                        self.loading = false;

                        self.error = response;
                    });
                }

                set_state();
            }
        }
    },
    methods: {
        onJoystickMoveRotate: function(x_value, y_value) {
            this.x_value = x_value;
        },
        onJoystickMoveTranslate: function(x_value, y_value) {
            this.y_value = y_value;
        }
    },
    mounted() {
        var self = this;

        async function get_state() {
            await fetch("http://0.0.0.0:8080/get_state", {
                method: "GET",
                headers: {
                    "Content-Type": "text/plain; charset=UTF-8"
                }
            })
            .then(function(response) {
                return response.text();
            })
            .then(function(response) {
                console.log("SUCCESS: " + response);
    
                self.state = response;
            })
            .catch(function(response) {
                console.log("FAILURE: " + response);

                self.error = response;
            });
        };

        get_state();
    },
    template: `
        <v-app id="app">
            <v-container>
                <div v-if="error !== undefined">
                    <v-row>
                        <v-col>
                            <h1>
                                ASDR REST API returned the following error: 
                            </h1>
                        </v-col>
                    </v-row>
                    <v-row>
                        <v-col>
                            <h2>
                                {{ error }}
                            </h2>
                        </v-col>
                    </v-row>
                </div>
                <div v-else>
                    <v-row>
                        <v-col>
                        </v-col>
                        <v-col cols="10">
                            <v-row>
                                <v-col>
                                    <h1>
                                        Autonomous Surface Disinfection Robot
                                    </h1>
                                </v-col>
                            </v-row>
                            <v-row class="my-16">
                                <v-col>
                                    <v-btn-toggle tile group color="deep-purple accent-3" v-model="state">
                                        <v-btn value="Idle">
                                            Idle
                                        </v-btn>
                                        <v-btn value="Manual">
                                            Manual
                                        </v-btn>
                                        <v-btn value="Automatic">
                                            Automatic
                                        </v-btn>
                                    </v-btn-toggle>
                                </v-col>
                            </v-row>
                            <v-row class="my-16">
                                <v-col>
                                    <div v-if="loading === true">
                                        <v-row>
                                            <v-col>
                                                <v-progress-circular color="deep-purple" :size="50" :width="5" indeterminate>
                                                </v-progress-circular>
                                            </v-col>
                                        </v-row>
                                    </div>
                                    <div v-else-if="state === 'Idle'">
                                    </div>
                                    <div v-else-if="state === 'Manual'">
                                        <v-row>
                                            <v-col>
                                                <Joystick ref="joystick_rotate" :x_max="1.0" :y_max="0.0" @joystickMove="onJoystickMoveRotate">
                                                </Joystick>
                                            </v-col>
                                            <v-col>
                                                <Joystick ref="joystick_translate" :x_max="0.0" :y_max="1.0" @joystickMove="onJoystickMoveTranslate">
                                                </Joystick>
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
                </div>
            </v-container>
        </v-app>
        `
};