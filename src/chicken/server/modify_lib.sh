#!/bin/bash

sed -i "1s/^/var EventEmitter2 = require('eventemitter2').EventEmitter2\n/" node_modules/mjpegcanvas/build/mjpegcanvas.js
echo -e "\nexport default MJPEGCANVAS" >> node_modules/mjpegcanvas/build/mjpegcanvas.js
echo -e "\nexport default ROSLIB" >> node_modules/roslib/build/roslib.js
