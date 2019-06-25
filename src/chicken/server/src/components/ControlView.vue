<template>
  <div class="controlview">
    <!-- joystick -->
    <div id="zone-joystick" class="zone active" style="touch-action: none;">
      <div class="view">
        <!-- stream -->
        <div class="stream">
          <div id="mjpeg-stream">
          </div>
        </div>
        <!-- ros 3d -->
      </div>
      <div class="info">
        <vs-list>
          <vs-list-header title="Motor Control"></vs-list-header>
          <vs-list-item title="Joystick Value">
            {{ valJoystick[0] | numFilter2Int }}, {{ valJoystick[1] |numFilter2Int }}
          </vs-list-item>
          <vs-list-item title="PWM Output">
            {{ valMotor[0] | numFilter2Int }}, {{ valMotor[1] | numFilter2Int }}
          </vs-list-item>
          <!-- <vs-list-header title="ROS" color="success"></vs-list-header>
          <vs-list-item title="Rosbag Recording"></vs-list-item>
          <vs-list-item title="XXXX" subtitle="YYYY"></vs-list-item> -->
        </vs-list>
      </div>  
    </div>
  </div>
</template>

<script>
import Vue from 'vue'
import nipplejs from 'nipplejs/dist/nipplejs'
import joystick2motor from '@/joystick2motor'
import MJPEGCANVAS from 'mjpegcanvas/build/mjpegcanvas'
import ROSLIB from 'roslib/build/roslib'
import { vsList } from 'vuesax'

Vue.use(vsList)

export default {
  name: 'ControlView',
  props: {
    msg: String
  },
  data: function () {
    return {
      sizeRatioCanvas: 0.75,
      scale: 0.5,
      maxWidthCanvas: 640,
      widthScreen: document.body.clientWidth,
      viewer: null,
      valMotor: [0, 0],
      valJoystick: [0, 0]
    }
  },
  filters: {
    numFilter2Int: function (val) {
      return parseFloat(val).toFixed(0)
    }
  },
  mounted: function () {
    this.init()
    this.createNipple()
    
    var vm = this
    window.onresize = () => {
      vm.widthScreen = document.body.clientWidth
      
      var w = this.widthScreen * this.scale
      w = w < this.maxWidthCanvas ? w : this.maxWidthCanvas
      var h = w * vm.sizeRatioCanvas
      vm.viewer.width = w
      vm.viewer.canvas.width = w
      vm.viewer.height = h
      vm.viewer.canvas.height = h
    }
  },
  methods: {
    init() {
      var vm = this
      // Create the canvas viewer.
      var w = this.widthScreen * this.scale
      w = w < this.maxWidthCanvas ? w : this.maxWidthCanvas
      this.viewer = new MJPEGCANVAS.Viewer({
        divID : 'mjpeg-stream',
        host : vm.$parent.ip,
        width : w,
        height : w * this.sizeRatioCanvas,
        topic : '/cam0/image_raw'
      })
    },
    createNipple() {
      var vm = this
      var diameter_joystick = 250
      var options = {
        zone: document.getElementById('zone-joystick'),
        color: '#FF6612',
        size: diameter_joystick,
        mode: 'semi',
        catchDistance: 300
      };
      var manager = nipplejs.create(options);

      // Get joystick data
      var norm = 2 * 127 / diameter_joystick
      manager.on('move', function(event, data) {
        var forward = data.distance * Math.sin(data.angle.radian) * norm,
            right = data.distance * Math.cos(data.angle.radian) * norm
        vm.valJoystick = [forward, right]

        // convert joystick value to motor's value. Both: [-128, 127]
        vm.valMotor = joystick2motor(right, forward)

        var topic_pwm = new ROSLIB.Topic({
          ros: vm.$parent.ros,
          name: '/motor/pwm_value',
          messageType: 'geometry_msgs/Point32'
        })
        var msg_pwm = new ROSLIB.Message({
          x: vm.valMotor[0],   // L
          y: vm.valMotor[1],   // R
          z: 0                    // null
        })
        topic_pwm.publish(msg_pwm)
      })
    }
  }
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
h3 {
  margin: 40px 0 0;
}
ul {
  list-style-type: none;
  padding: 0;
}
li {
  display: inline-block;
  margin: 0 10px;
}
a {
  color: #42b983;
}
.zone {
  display: none;
  position: absolute;
  width: 100%;
  height: 100%;
  left: 0;
}
.zone.active {
  display: block;
}
.zone > h1 {
  position: absolute;
  padding: 10px 10px;
  margin: 0;
  color: white;
  right: 0;
  bottom: 0;
}
.zone.nipple {
  position: relative !important;
}
.view {
  position: relative;

}
.view .stream {
  position: relative;
  /* related to variable scale */
  left: -25%;
}
.view .info {
  position: relative;
  text-align: left;
}
</style>
