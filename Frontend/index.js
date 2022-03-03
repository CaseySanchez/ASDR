import Joystick from "./joystick.js"

export default {
    name: "index",
    components: {
        Joystick
    },
    data() {
        return {
            state: "Idle"
        };
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
                        <v-row>
                            <h1>Autonomous Surface Disinfection Robot</h1>
                        </v-row>
                        <v-row>
                            <v-btn-toggle tile group color="deep-purple accent-3" v-model="state">
                                <v-btn value="Idle">Idle</v-btn>
                                <v-btn value="Manual">Manual</v-btn>
                                <v-btn value="Automatic">Automatic</v-btn>
                            </v-btn-toggle>
                        </v-row>
                        <v-row>
                            <v-col>
                                <Joystick ref="joystick_rotate" :width="200" :height="200" :x_max="1.0" :y_max="0.0"></Joystick>
                            </v-col>
                            <v-col>
                                <Joystick ref="joystick_translate" :width="200" :height="200" :x_max="0.0" :y_max="1.0"></Joystick>
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