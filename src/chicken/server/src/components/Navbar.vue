<template>
  <div>
    <vs-navbar
    v-model="activeItem"
    color="#212730"
    text-color="rgba(255,255,255,.6)"
    active-text-color="rgba(255,255,255,1)"
    class="nabarx">
      <div slot="title">
        <vs-navbar-title id="title">
          {{ title }}
        </vs-navbar-title>
      </div>

      <vs-navbar-item index="0">
        <p>Home</P>
      </vs-navbar-item>
      
      <img id="record-img" :src="recordImg" alt="RecordButton" v-on:click="recordButton"/>
      
    </vs-navbar>
  </div>
</template>

<script>
import Vue from 'vue'
import { vsNavbar } from 'vuesax'
import 'vuesax/dist/vuesax.css' // Vuesax styles
Vue.use(vsNavbar)
import ROSLIB from 'roslib/build/roslib'

export default {
  name: 'navbar',
  data: function () {
    return {
      activeItem: 0,
      title: 'ROBOT304',
      recordText: 'Record Button',
      recordImg: require('@/assets/recordOff.png'),
      recording: false,
    }
  },
  methods: {
    recordButton() {
      var vm = this
      var status = !this.recording
      var srv_record = new ROSLIB.Service({
        ros: vm.$parent.ros,
        name: '/record/cmd',
        serviceType: 'record_ros/String_cmd'
      })

      var req_record = new ROSLIB.ServiceRequest({
        cmd: status ? 'record' : 'stop'
      })
      srv_record.callService(req_record, 
      function(res) {
        var msg = JSON.stringify(res.res)
        if (JSON.stringify("starting recorder") === msg) {
          vm.recording = true
        }
        else if (JSON.stringify("stopping recorder") === msg) {
          vm.recording = false
        }

        if (vm.recording) {
          vm.recordImg = require('@/assets/recordOn.png')
          vm.recordText = "Recording..."
        } else {
          vm.recordImg = require('@/assets/recordOff.png')
          vm.recordText = "Record Button"
        }
      })
      // function (err) {
      //   console.error("[ERR]: " + err);
      // })
    }
  }
}
</script>

<style scope>
.nabarx {
  color: rgb(255, 255, 255);
  padding: 15px;
  padding-right: 15px;
}
#title {
  font-size: 3em;
  font-weight: 700;
}
.vs-navbar--btn-responsive .btn-responsive-line {
  background: rgb(255, 255, 255);
}
#record-img {
  height: 50px;
  width: 50px;
  margin-left: 10px;
}
</style>