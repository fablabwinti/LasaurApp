'use strict';

var app = angular.module("LasaurAdmin", ['ui.bootstrap', 'angularSpinner'])
app.controller('AdminController', function ($scope, $http, $log) {
    var vm = this;

    vm.firmware_version = 'not asked yet';
    vm.config = {};
    vm.busy = false;
    vm.message = '';

    vm.flashFirmware = function(use_prebuilt_release) {
        var url = '/firmware/flash';
        if (use_prebuilt_release) {
            url += '_release';
        }
        vm.busy = true;
        vm.message = 'Flashing... (takes about 10 seconds)';
        return $http.post(url).then(
            function success(resp) {
                vm.busy = false;
                vm.message = 'Success!';
            }, function error(resp) {
                vm.busy = false;
                vm.message = 'flash failed: ' + resp.statusText + ' ' + resp.data;
            }
        );
    };

    vm.flashRelease = function() {
        vm.flashFirmware(true);
    }

    vm.buildAndFlash = function() {
        vm.busy = true;
        vm.message = 'Building firmware...';
        return $http.post('/firmware/build').then(
            function success(resp) {
                vm.flashFirmware(false);
            }, function error(resp) {
                vm.busy = false;
                vm.message = 'build failed: ' + resp.statusText + ' ' + resp.data;
            }
        );
    };

    vm.resetFirmware = function() {
        vm.busy = true;
        vm.message = 'Reset...';
        return $http.post('/firmware/reset').then(
            function success(resp) {
                vm.busy = false;
                vm.message = 'Success!';
            }, function error(resp) {
                vm.busy = false;
                vm.message = 'reset failed: ' + resp.statusText + ' ' + resp.data;
            }
        );
    };

    $http.get('/config').then(function(response) {
        var config = response.data;
        //$scope.work_area_width = config.workspace[0];
        //$scope.work_area_height = config.workspace[1];
        vm.config = config;
    });
})
