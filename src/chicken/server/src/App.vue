<template>
  <div id="app">
    <Navbar/>
    <ControlView/>
  </div>
</template>

<script>
import Navbar from '@/components/Navbar'
import ControlView from '@/components/ControlView'
import ROSLIB from 'roslib/build/roslib'

export default {
  name: 'app',
  components: {
    Navbar,
    ControlView
  },
  data: function () {
    return {
      ip: null,
      ros: null,
    }
  },
  beforeMount: function () {
    var vm = this
    this.ip = window.location.hostname
    this.ip = (undefined === this.ip || "" === this.ip) ? 'localhost' : this.ip

    this.ros = new ROSLIB.Ros({
      url: "ws://" + vm.ip + ":9090"
    })
  }
}
</script>

<style>
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  /* margin-top: 60px; */
}
</style>
