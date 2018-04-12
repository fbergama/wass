


$(document).ready( function() {

    var refresh_workspace_data = function() {
        $.getJSON("worksession", function(data) {
            $("#nframesvar").html( data.wdir_frames.length );
            $("#wdirvar").html( data.workdir );
            $("#cam0dirvar").html( data.cam0_datadir );
            $("#cam1dirvar").html( data.cam1_datadir );
            $("#savediskspacevar").html( data.savediskspace ? "Yes" : "No" );
            $("#keepimagesvar").html( data.keepimages ? "Yes" : "No" );
        });
    }

    var add_activity = function( name, percent) {

        $('#jobspanel').append('<div class="well well-sm activejob"><div class="taskname"><span class="glyphicon glyphicon-cog" aria-hidden="true">&nbsp;</span>'+name+'</div><div class="progress"><div class="progress-bar" role="progressbar" aria-valuenow="'+percent+'" aria-valuemin="0" aria-valuemax="100" style="width: '+percent+'%;">'+percent+'%</div></div></div>');

    }

    var show_alert = function( message ) {
        $("#alerts").append('<div class="alert alert-danger alert-dismissible" role="alert"><button type="button" class="close" data-dismiss="alert" aria-label="Close"><span aria-hidden="true">&times;</span></button>'+message+'</div');
    }

    var time_to_string = function( t ) {
        var seconds = Math.round(t/1000.0);

        if( seconds == 0 )
            return "-";

        var minutes = Math.floor(seconds/60.0) % 60;
        var hours = Math.floor( seconds/3600.0 ) % 24;
        var days = Math.floor( seconds/86400.0 );

        return ((days>0)?(days+" day(s), "):"")+ ((hours>0)?(hours+" hour(s), "):"")+ ((minutes>0)?(minutes+" minute(s), "):"") + (seconds%60) +" second(s)";
    }


    $("#preparebutton").on("click", function() {
        $.getJSON("doprepare", function(data) {
            if( !data.success ) {
                show_alert(data.reason);
            }
        });
    });
    $("#matchbutton").on("click", function() {
        $.getJSON("domatch", function(data) {
            if( !data.success ) {
                show_alert(data.reason);
            }
        });
    });
    $("#matchmergebutton").on("click", function() {
        $.getJSON("domatchmerge", function(data) {
            if( !data.success ) {
                show_alert(data.reason);
            }
        });
    });
    $("#extloadbutton").on("click", function() {
        $.getJSON("doextload", function(data) {
            if( !data.success ) {
                show_alert(data.reason);
            }
        });
    });
    $("#densebutton").on("click", function() {
        $.getJSON("dodense", function(data) {
            if( !data.success ) {
                show_alert(data.reason);
            }
        });
    });
    $("#workspacereloadbutton").on("click", function() {
        $.getJSON("reloadworksession", function(data) {
            if( data.error ) {
                show_alert("Workspace reload failed");
            }
            refresh_workspace_data();
        }).error("Workspace reload failed").done();
    });
    $("#playbutton").on("click", function() {
        $.getJSON("resumequeue", function(data) {
            if( data.error ) {
                show_alert("Queue start failed");
            }
        }).error("Queue start failed").done();
    });
    $("#pausebutton").on("click", function() {
        $.getJSON("pausequeue", function(data) {
            if( data.error ) {
                show_alert("Queue start failed");
            }
        }).error("Queue start failed").done();
    });


    setInterval( function() {

        $.getJSON("activejobs", function(data) {
            $("#jobspanel").html("");
            $("#numrunningjobs").text( ""+data.length );
            data.forEach( function(job) {
                add_activity( job.data.title, job.progress);
            });
        });

        $.getJSON("fullstatus", function(data) {

            $("#statuslabel").text(data.current_status);

            // completed tasks
            //
            $("#isprepared").css("display", data.completed_tasks.prepare?"inline":"none");
            $("#ismatched").css("display", data.completed_tasks.match?"inline":"none");
            $("#ismerged").css("display", data.completed_tasks.matchmerge?"inline":"none");
            $("#isextloaded").css("display", data.completed_tasks.extload?"inline":"none");
            $("#isreconstructed").css("display", data.completed_tasks.dense?"inline":"none");


            // progress percent
            //
            if( data.progress_max===0 )
                data.progress_max = 1;
            var percent = Math.round(data.progress / data.progress_max * 100.0) + "%";

            $("#main_progress").css("width", percent);
            $("#main_progress").text(percent);

            $("#remaining_time").text( time_to_string(data.remaining) );

            if( data.num_failed > 0 ) {
                $("#failures").css("display","inline");
                $("#numfailures").text( data.num_failed );
            } else {
                $("#failures").css("display","none");
            }

            if( data.progress == 0 )
                $("#workspacepanel").hide(400);
            else
                $("#workspacepanel").show(400);

        });
    }, 500 );

    setTimeout( refresh_workspace_data, 500);

});
