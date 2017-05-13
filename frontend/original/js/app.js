var new_api = 'http://' + window.location.hostname + ':5555';

var hardware_ready_state = false;
var firmware_version_reported = false;
var progress_not_yet_done_flag = false;
var previous_error_report = '';
var stop_and_resume_in_progress = false;
var last_command_was_homing = false;

(function($){
  $.fn.uxmessage = function(kind, text, max_length) {
    if (max_length == null) {
      max_length = 100;
    }

    if (text.length > max_length) {
      text = text.slice(0,max_length) + '\n...'
    }

    text = text.replace(/\n/g,'<br>')
    
    if (kind == 'notice') {
      $('#log_content').prepend('<div class="log_item log_notice well" style="display:none">' + text + '</div>');
      $('#log_content').children('div').first().show('blind');
      if ($("#log_content").is(':hidden')) {
        $().toastmessage('showNoticeToast', text);
      }
    } else if (kind == 'success') {
      $('#log_content').prepend('<div class="log_item log_success well" style="display:none">' + text + '</div>');
      $('#log_content').children('div').first().show('blind');
      if ($("#log_content").is(':hidden')) {
        $().toastmessage('showSuccessToast', text);   
      }
    } else if (kind == 'warning') {
      $('#log_content').prepend('<div class="log_item log_warning well" style="display:none">' + text + '</div>');
      $('#log_content').children('div').first().show('blind');
      if ($("#log_content").is(':hidden')) {
        $().toastmessage('showWarningToast', text);   
      }
    } else if (kind == 'error') {
      $('#log_content').prepend('<div class="log_item log_error well" style="display:none">' + text + '</div>');
      $('#log_content').children('div').first().show('blind');
      if ($("#log_content").is(':hidden')) {
        $().toastmessage('showErrorToast', text);   
      }
    }

    while ($('#log_content').children('div').length > 200) {
      $('#log_content').children('div').last().remove();
    }

  };
})(jQuery); 


function send_gcode(gcode, success_msg, progress) {
  if (gcode.indexOf('G30\n') !== -1) {
    last_command_was_homing = true;
  } else {
    last_command_was_homing = false;
  }
  // if (hardware_ready_state || gcode[0] == '!' || gcode[0] == '~') {
  if (true) {
    if (typeof gcode === 'string' && gcode != '') {
      // $().uxmessage('notice', gcode, Infinity);

      if (progress == true) {
        // show progress bar, register live updates
        if ($("#progressbar").children().first().width() == 0) {
          $("#progressbar").children().first().width('2%');
          $("#progressbar").show();
          progress_not_yet_done_flag = true;
          setTimeout(update_progress, 2000);
        }
      }

      $.ajax({
        type: 'POST',
        url: new_api + '/gcode',
        contentType: 'text/plain',
        data: gcode,
        dataType: 'json',
        success: function (data) {
          if (previous_error_report && gcode[0] != '!' && gcode[0] != '~') {
            $().uxmessage('error', 'Error unresolved: ' + previous_error_report);
          }
          $().uxmessage('success', success_msg);
        },
        error: function (data) {
          if (data.responseText) {
            $().uxmessage('error', 'Error posting gcode: ' + data.responseText);
          } else {
            $().uxmessage('error', 'Error posting gcode. LasaurApp server down?');
          }
          console.log(data);
        },
        complete: function (data) {
          // future use
        }
      });
    } else {
      $().uxmessage('error', "No gcode.");
    }
  } else {
    $().uxmessage('warning', "Not ready, request ignored.");
  }
}


function update_progress() {
  $.get(new_api + '/status', function(data) {
    var pct = data.queue.job_percent;
    var busy = !data.ready;
    if (pct != 100 || busy) {
      $("#progressbar").show();
      var pct_gui = 2 + pct/100*94;
      $("#progressbar").children().first().width(pct_gui+'%');
      setTimeout(update_progress, 2000);
    } else {
      if (progress_not_yet_done_flag) {
        $("#progressbar").show();
        $("#progressbar").children().first().width('100%');
        $().uxmessage('notice', "Done.");
        progress_not_yet_done_flag = false;
        setTimeout(update_progress, 2000);
      } else {
        $('#progressbar').hide();
        $("#progressbar").children().first().width(0);
      }
    }
  });
}


function open_bigcanvas(scale, deselectedColors) {
  var w = scale * app_settings.canvas_dimensions[0];
  var h = scale * app_settings.canvas_dimensions[1];
  $('#container').before('<a id="close_big_canvas" href="#"><canvas id="big_canvas" width="'+w+'px" height="'+h+'px" style="border:1px dashed #aaaaaa;"></canvas></a>');
  var mid = $('body').innerWidth()/2.0-30;
  $('#close_big_canvas').click(function(e){
    close_bigcanvas();
    return false;
  });
  $("html").on('keypress.closecanvas', function (e) {
    if ((e.which && e.which == 13) || (e.keyCode && e.keyCode == 13) ||
        (e.which && e.which == 27) || (e.keyCode && e.keyCode == 27)) {
      // on enter or escape
      close_bigcanvas();
      return false;
    } else {
      return true;
    }
  });
  // $('#big_canvas').focus();
  $('#container').hide();
  var bigcanvas = new Canvas('#big_canvas');
  // DataHandler.draw(bigcanvas, 4*app_settings.to_canvas_scale, getDeselectedColors());
  if (deselectedColors === undefined) {
    DataHandler.draw(bigcanvas, scale*app_settings.to_canvas_scale);
  } else {
    DataHandler.draw(bigcanvas, scale*app_settings.to_canvas_scale, deselectedColors);
  }
}


function close_bigcanvas() {
  $('#big_canvas').remove();
  $('#close_big_canvas').remove();
  $('html').off('keypress.closecanvas');
  delete bigcanvas;
  $('#container').show();
}


function generate_download(filename, filedata) {
  $.ajax({
    type: "POST",
    url: "/stash_download",
    data: {'filedata': filedata},
    success: function (data) {
      window.open("/download/" + data + "/" + filename, '_blank');
    },
    error: function (data) {
      console.log(data);
      $().uxmessage('error', "Error. LasaurApp server down?");
    },
    complete: function (data) {
      // future use
    }
  });
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



$(document).ready(function(){
  
  $().uxmessage('notice', "Frontend started.");

  $.getJSON('/version', function(data) {
    $().uxmessage('notice', "LasaurApp v" + data.VERSION);
    $('#lasaurapp_version').html(data.VERSION);
  });

  $('#feedrate_field').val(app_settings.max_seek_speed);

  $('#tab_logs_button').click(function(){
    $('#log_content').show()
    $('#tab_logs div.alert').show()
  });

  //////// serial connect and pause button ////////
  var pause_btn_state = false;

  function connect_btn_set_state(is_connected) {
    if (is_connected) {
      $("#connect_btn").removeClass("btn-danger");
      $("#connect_btn").removeClass("btn-warning");
      $("#connect_btn").addClass("btn-success");      
    } else {
      $("#connect_btn").removeClass("btn-danger");
      $("#connect_btn").removeClass("btn-success");
      $("#connect_btn").addClass("btn-warning");     
    }
  }
    
  // get hardware status
  var previous_status = {};
  function poll_hardware_status() {
    function report_error(error_report) {
      if (error_report != previous_error_report) {
        previous_error_report = error_report;
        if (error_report) {
          $().uxmessage('error', 'Error: ' + error_report);
          $("#tab_logs_span").addClass("label label-warning");
          $("#cancel_btn").addClass("btn-info");
          $("#homing_cycle").addClass("btn-info");
        } else {
          $().uxmessage('success', 'Error resolved');
          $("#tab_logs_span").removeClass("label label-warning");
          $("#cancel_btn").removeClass("btn-info");
          $("#homing_cycle").removeClass("btn-info");
        }
      }
    }
    $.getJSON(new_api + '/status', function(status) {
      // avoid flickering GUI elements while idle
      if (JSON.stringify(status) !== JSON.stringify(previous_status)) {
        previous_status = status;

        // pause status
        if (status.paused) {
          pause_btn_state = true;
          $("#pause_btn").addClass("btn-primary");
          $("#pause_btn").html('<i class="icon-play"></i>');
        } else {
          pause_btn_state = false;
          $("#pause_btn").removeClass("btn-warning");
          $("#pause_btn").removeClass("btn-primary");
          $("#pause_btn").html('<i class="icon-pause"></i>');
        }

        if (status.serial_connected) {
          connect_btn_set_state(true);
        } else {
          connect_btn_set_state(false);
        }

        // ready state
        if (status.ready) {
          hardware_ready_state = true;
          $("#connect_btn").html("Ready");
        } else {
          if (status.serial_connected) {
            $("#connect_btn").html("Busy");
          }
          hardware_ready_state = false;
        }

        // door, chiller, power, limit, buffer
        if (status.serial_connected) {
          if (status.info.door_open) {
            $('#door_status_btn').removeClass('btn-success')
            $('#door_status_btn').addClass('btn-warning')
            // $().uxmessage('warning', "Door is open!");
          } else {
            $('#door_status_btn').removeClass('btn-warning')
            $('#door_status_btn').addClass('btn-success')
          }
          if (status.info.chiller_off) {
            $('#chiller_status_btn').removeClass('btn-success')
            $('#chiller_status_btn').addClass('btn-warning')
            // $().uxmessage('warning', "Chiller is off!");
          } else {
            $('#chiller_status_btn').removeClass('btn-warning')
            $('#chiller_status_btn').addClass('btn-success')
          }
          // if (status.power_off) {
          //   $().uxmessage('error', "Power is off!");
          //   $().uxmessage('notice', "Turn on Lasersaur power then run homing cycle to reset.");
          // }
          if (status.pos.x && status.pos.y) {
            // only update if not manually entering at the same time
            if (!$('#x_location_field').is(":focus") &&
                !$('#y_location_field').is(":focus") &&
                !$('#location_set_btn').is(":focus") &&
                !$('#origin_set_btn').is(":focus"))
            {
              var x = parseFloat(status.pos.x).toFixed(2)/1;
              $('#x_location_field').val(x.toFixed(2));
              $('#x_location_field').animate({
                opacity: 0.5
              }, 100, function() {
                $('#x_location_field').animate({
                  opacity: 1.0
                }, 600, function() {});
              });
              var y = parseFloat(status.pos.y).toFixed(2)/1;
              $('#y_location_field').val(y.toFixed(2));
              $('#y_location_field').animate({
                opacity: 0.5
              }, 100, function() {
                $('#y_location_field').animate({
                  opacity: 1.0
                }, 600, function() {});
              });
            }
          }
          if (status.firmver && !firmware_version_reported) {
            $().uxmessage('notice', "Firmware v" + status.firmver);
            $('#firmware_version').html(status.firmver);
            firmware_version_reported = true;
          }
        }
        var msg = status.error_report;
        if (stop_and_resume_in_progress) {
          // known cause, don't report
          if (msg === 'stopped - serial_stop_request') {
            msg = '';
          }
        }
        if (last_command_was_homing) {
          // known cause, no status updates during homing
          if (msg === 'last status update from driveboard is too old') {
            msg = '';
          }
        }
        report_error(msg);
      }
      // schedule next hardware poll
      setTimeout(function() {poll_hardware_status()}, 300);
    }).fail(function() {
      // lost connection to server
      previous_status = {};
      report_error("connection to backend webserver lost");
      connect_btn_set_state(false);
      // schedule next hardware poll
      setTimeout(function() {poll_hardware_status()}, 1000);
    });
  }
  // kick off hardware polling
  poll_hardware_status();

  connect_btn_width = $("#connect_btn").innerWidth();
  $("#connect_btn").width(connect_btn_width);

  $("#pause_btn").tooltip({placement:'bottom', delay: {show:500, hide:100}});
  $("#pause_btn").click(function(e){
    console.log('click stat', pause_btn_state)
    if (pause_btn_state == true) {  // unpause
      send_gcode('!unpause', 'Continuing...', false);
    } else {  // pause
      $("#pause_btn").addClass('btn-warning');
      send_gcode('!pause', 'Pausing in a bit...', false);
    }
    e.preventDefault();   
  }); 
  //\\\\\\ serial connect and pause button \\\\\\\\
  
  
  $("#cancel_btn").tooltip({placement:'bottom', delay: {show:500, hide:100}});
  $("#cancel_btn").click(function(e){
    stop_and_resume_in_progress = true;
    var gcode = '!\n'  // ! is enter stop state char
    // $().uxmessage('notice', gcode.replace(/\n/g, '<br>'));
    send_gcode(gcode, "Stopping ...", false); 
    var delayedresume = setTimeout(function() {
      var gcode = '~\nG90\nG0X0Y0F'+app_settings.max_seek_speed+'\nM81\n'  // ~ is resume char
      // $().uxmessage('notice', gcode.replace(/\n/g, '<br>'));
      send_gcode(gcode, "Resetting ...", false);
      setTimeout(function() { stop_and_resume_in_progress = false; }, 1000);
    }, 1000);
    e.preventDefault();   
  });
  
  $("#homing_cycle").tooltip({placement:'bottom', delay: {show:500, hide:100}});
  $("#homing_cycle").click(function(e){
    stop_and_resume_in_progress = true;
    var gcode = '!\n'  // ! is enter stop state char
    // $().uxmessage('notice', gcode.replace(/\n/g, '<br>'));
    send_gcode(gcode, "Resetting ...", false);
    var delayedresume = setTimeout(function() {
      var gcode = '~\nG30\n'  // ~ is resume char
      // $().uxmessage('notice', gcode.replace(/\n/g, '<br>'));
      send_gcode(gcode, "Homing cycle ...", false);
      setTimeout(function() {
        stop_and_resume_in_progress = false;
      }, 1000);
    }, 1000);
    e.preventDefault(); 

  });

  $("#go_to_origin").tooltip({placement:'bottom', delay: {show:500, hide:100}});
  $("#go_to_origin").click(function(e){
    var gcode;
    if(e.shiftKey) {
      // also reset offset
      reset_offset();
    }
    gcode = 'G90\nG0X0Y0F'+app_settings.max_seek_speed+'\n'
    // $().uxmessage('notice', gcode);
    send_gcode(gcode, "Going to origin ...", false);
    e.preventDefault();
  });

  /// tab shortcut keys /////////////////////////
  $(document).on('keypress', null, 'p', function(e){
    $('#pause_btn').trigger('click');
    return false;
  });

  $(document).on('keypress', null, '0', function(e){
    $('#go_to_origin').trigger('click');
    return false;
  });

  var cancel_modal_active = false;
  $(document).on('keyup', null, 'esc', function(e){
    if (cancel_modal_active === true) {
      $('#cancel_modal').modal('hide');
      cancel_modal_active = false;
    } else {
      $('#cancel_modal').modal('show');
      $('#really_cancel_btn').focus();
      cancel_modal_active = true;
    }
    return false;
  });

  $('#really_cancel_btn').click(function(e){
    $('#cancel_btn').trigger('click');
    $('#cancel_modal').modal('hide');
    cancel_modal_active = false;
  });



  /// tab shortcut keys /////////////////////////

  $(document).on('keypress', null, 'j', function(e){
    $('#tab_jobs_button').trigger('click');
    return false;
  });

  $(document).on('keypress', null, 'i', function(e){
    $('#tab_import_button').trigger('click');
    return false;
  });

  $(document).on('keypress', null, 'm', function(e){
    $('#tab_mover_button').trigger('click');
    return false;
  });

  $(document).on('keypress', null, 'l', function(e){
    $('#tab_logs_button').trigger('click');
    return false;
  });
  
});  // ready
