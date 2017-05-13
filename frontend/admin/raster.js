'use strict';

angular.module('app.raster', ['app.core'])
.controller('RasterController', function ($scope, $http, $websocket, $timeout, $interval, RasterLib) {
    var vm = this;

    const LASER_POWER = 100.0; // watts
    const PULSE_SECONDS = 31.875e-6;  // see laser.c
    const MINIMUM_PULSE_TICKS = 3;    // unit: PULSE_SECONDS
    const MAXIMUM_PULSE_TICKS = 127;  // unit: PULSE_SECONDS
    const ACCELERATION = 1800000.0    // mm/min^2, divide by (60*60) to get mm/sec^2
    const BAUDRATE = 57600;

    vm.uploadedImage = null;

    vm.requested_ppmm = 7.0;
    vm.requested_pulse = null;
    vm.requested_lead_in = 2.5;
    vm.max_feedrate = 6000;
    vm.max_intensity = 80;

    vm.params = {
        width: 50.0,
        energy_density: 0.4,
        invert: false,
        // binary: false,
        bidirectional: true,
        skip_empty: true,
        lead_in: 2.0,
        pos_x: 0.0,
        pos_y: 0.0
    }

    vm.frame_enable = false;
    vm.frame_cut_feedrate = 1500;
    vm.frame_cut_intensity = 70;

    vm.pulse_duration_us = function() {
        return vm.params.pulse * PULSE_SECONDS / 1e-6;
    }

    vm.recalculate = function() {
        // Current method: (optimized towards bi-level images)
        // - choose a pulse_duration
        // - adjust ppmm such that black pixels have the correct energy_density
        // - scale image according to ppmm
        // - use dithering for mid-tones
        // - decrease feedrate if required to implement high energy_density or high ppmm

        // Calculate ppmm for each allowed pulse duration,
        // and choose the ppmm closest to the target.
        var best_pulse;
        var best_ppmm;
        (function find_best_params() {
            var best_error;
            for (var pulse = MINIMUM_PULSE_TICKS; pulse <= MAXIMUM_PULSE_TICKS; pulse++) {
                var pulse_duration = pulse * PULSE_SECONDS;
                var energy_per_pulse = pulse_duration * LASER_POWER;  // joules
                var pulse_density = vm.params.energy_density / energy_per_pulse;
                var ppmm = Math.sqrt(pulse_density);
                var error = Math.abs(vm.requested_ppmm - ppmm);
                if (pulse === MINIMUM_PULSE_TICKS || error < best_error) {
                    best_pulse = pulse;
                    best_ppmm = ppmm;
                    best_error = error;
                }
            }
        })();

        var pulse = best_pulse;
        var ppmm = best_ppmm;
        vm.params.pulse = pulse;
        vm.params.ppmm = ppmm;

        var feedrate = vm.max_feedrate;
        vm.params.travel_feedrate = feedrate;

        // limit feedrate such that max_intensity is respected
        var feedrate_limit1 = vm.max_intensity/100.0 / (ppmm * (pulse * PULSE_SECONDS)) * 60;
        feedrate = Math.min(feedrate, feedrate_limit1);

        // limit feedrate such that the driveboard will not slow down to wait for serial data
        var byte_per_second = BAUDRATE/10/2;  // (8 bits + startbit + stopbit) and every byte is sent twice (error detection)
        byte_per_second *= 0.5  // leave some room for parameters and line commands (just a rough guess)
        var feedrate_limit2 = byte_per_second / ppmm * 60.0
        feedrate = Math.min(feedrate, feedrate_limit2);

        vm.params.raster_feedrate = feedrate;

        vm.actual_intensity = (pulse * PULSE_SECONDS) * ( (feedrate/60.0) * ppmm )  * 100;

        RasterLib.makeGrayScale();
        var line_count = RasterLib.makePulseImage(vm.params);
        updatePulsePreview();
        updateGrayScalePreview();

        // estimate duration
        var accel_dist = 0.5 * Math.pow(feedrate, 2) / ACCELERATION;
        vm.accel_dist = accel_dist;

        // Limit lead-in to the acceleration time plus 3 seconds.
        var lead_in_max = accel_dist + 3.0 * feedrate / 60;
        vm.params.lead_in = Math.min(vm.requested_lead_in, lead_in_max);

        var line_length = 2*vm.params.lead_in + vm.params.width;
        var cruising_dist = line_length - 2*accel_dist;
        if (cruising_dist <= 0) {
            cruising_dist = 0;
            accel_dist = line_length/2;
        }
        var cruising_time = cruising_dist / feedrate;
        var accel_time = Math.sqrt(2*accel_dist / ACCELERATION);
        var line_duration = cruising_time + 2*accel_time;
        vm.duration = line_duration * line_count;
        if (!vm.params.bidirectional) vm.duration *= 2;

        vm.height_calculated = vm.params.width/vm.uploadedImage.width*vm.uploadedImage.height;
        vm.height_calculated_pretty = vm.height_calculated.toFixed(1);
    }

    vm.sendGcode = function(gcode) {
        return $http({
            method: 'POST',
            url: '/gcode',
            data: gcode,
            headers: {
                'Content-Type': 'text/plain'
            }
        }).then(function success(resp) {
            vm.submitStatus = 'Gcode sent to backend.';
            $timeout(function() { vm.submitStatus = ''; }, 3000);
        }, function error(resp) {
            vm.submitStatus = 'Error sending gcode to backend.';
            $timeout(function() { vm.submitStatus = ''; }, 3000);
            vm.serverMessage = resp.data;
        })
    }

    vm.sendJob = function () {
        var gcode_raster = RasterLib.makeGcode(vm.params);

        vm.submitStatus = 'Generating gcode...';
        vm.serverMessage = '';

        $timeout(function() {
            var gcode = '';
            // gcode += 'G30\n';  // homing
            gcode += 'M80\n';  // air_enable
            gcode += gcode_raster;

            if (vm.frame_enable) {
                var x0 = vm.params.pos_x;
                var y0 = vm.params.pos_y;
                var x1 = x0 + vm.params.width;
                var y1 = y0 + vm.height_calculated;
                gcode += cutBoxGcode(x0, y0, x1, y1);
            }

            gcode += 'G0 X0 Y0 F' + vm.params.travel_feedrate.toFixed(2) + '\n';
            gcode += 'M81\n';  // air_disable

            vm.submitStatus = 'Sending gcode...';
            vm.serverMessage = '';
            vm.sendGcode(gcode);
        });
    }

    function cutBoxGcode(x0, y0, x1, y1) {
        x0 = x0.toFixed(2);
        y0 = y0.toFixed(2);
        x1 = x1.toFixed(2);
        y1 = y1.toFixed(2);
        var f_travel = vm.params.travel_feedrate.toFixed(2);
        var f_cut = vm.frame_cut_feedrate.toFixed(2);
        var s_cut = (vm.frame_cut_intensity * 255 / 100).toFixed(2);
        var gcode = '';
        gcode += 'G0 X'+x0+' Y'+y0 + ' F'+f_travel + '\n';
        gcode += 'G1 S' + s_cut + ' F'+f_cut + '\n';
        gcode += 'G1 X'+x1+' Y'+y0 + '\n';
        gcode += 'G1 X'+x1+' Y'+y1 + '\n';
        gcode += 'G1 X'+x0+' Y'+y1 + '\n';
        gcode += 'G1 X'+x0+' Y'+y0 + '\n';
        return gcode;
    }

    vm.stopAndResume = function() {
        return vm.sendGcode('!').then(function() {  // stop
            return $timeout(function() {
                return vm.sendGcode('~');  // resume
            }, 1000);
        });
    };

    vm.homing = function() {
        vm.stopAndResume().then(function() {
            vm.sendGcode('G30');  // homing
        });
    };

    vm.goToOrigin = function() {
        vm.sendGcode('G0 X0 Y0 F' + vm.max_feedrate.toFixed(2));
    }

    vm.pause = function() {
        vm.sendGcode('!pause');
    }
    vm.unpause = function() {
        vm.sendGcode('!unpause');
    }

    // FIXME: status code duplication with admin.js. Maybe turn this into a service?
    vm.status = {};
    vm.showJson = false;
    vm.haveStatusUpdates = true;
    vm.lastStatusMessageReceived = true;

    var statusStream = $websocket('ws://' + window.location.host + '/status/ws');
    statusStream.onMessage(function(message) {
        vm.status = JSON.parse(message.data);
        vm.lastStatusMessageReceived = true;
        vm.haveStatusUpdates = true;
    });
    $interval(function watchStatusUpdates() {
        if (!vm.lastStatusMessageReceived) {
            vm.haveStatusUpdates = false;
        }
        vm.lastStatusMessageReceived = false;
    }, 1500);


    vm.canvas_gray = null;
    vm.canvas_pulse = null;

    document.getElementById('file-input').addEventListener('change', onFileChanged, false);
    function onFileChanged(changeEvent) {
        var reader = new FileReader();
        reader.onload = fileLoaded;
        reader.readAsDataURL(changeEvent.target.files[0]);
        function fileLoaded(loadEvent) {
            var img = new Image();
            img.onload = imageLoaded;
            img.src = loadEvent.target.result;
            function imageLoaded() {
                $scope.$apply(function() {
                    setImage(img);
                });
            }
        }
    }

    /*
    // for development/debugging
    function debugInit() {
        var img = new Image();
        img.onload = imageLoaded;
        img.src = 'img.jpg';
        function imageLoaded() {
            $scope.$apply(function() {
                setImage(img);
            });
        }
    }
    debugInit();
    */

    function setImage(img) {
        vm.uploadedImage = img;
        RasterLib.setImage(img);
        vm.recalculate();
    }

    function updateGrayScalePreview() {
        vm.canvas_gray = RasterLib.grayCanvas;
        if (vm.canvas_gray.modified) {
            vm.canvas_gray.modified += 1;
        } else {
            vm.canvas_gray.modified = 1;
        }
    }

    function updatePulsePreview() {
        vm.canvas_pulse = RasterLib.pulseCanvas;
        if (vm.canvas_pulse.modified) {
            vm.canvas_pulse.modified += 1;
        } else {
            vm.canvas_pulse.modified = 1;
        }
    }
})
