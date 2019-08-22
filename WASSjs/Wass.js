/*************************************************************************

WASS - Wave Acquisition Stereo System
Copyright (C) 2016  Filippo Bergamasco

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*************************************************************************/

require('events').EventEmitter.prototype._maxListeners = 0;
var kue = require('kue');
var RunTask = require("./runtask.js");
var utils = require("./utils.js");
var dirscan = require("./dirscan.js");
var Q = require('q');
var fs = require('fs');
var sprintf = require("sprintf-js").sprintf;
var ioRedis = require('ioredis');


var kuesettings = { redis:'redis://localhost:6379' }

if( process.env.REDISADDR ) {
    var kuesettings = { redis:'redis://'+process.env.REDISADDR }
}



var WASSjs_VERSION = "1.4";

console.log( "Welcome to Wass.js  - " + WASSjs_VERSION );
console.log( "======================================================" );




// Setup all handlers
process.on("exit", function() { console.log("Goodbye"); });




// Load settings file
var settings = undefined;
function load_settings() {
    try {
        settings = require("./settings.json");
        settings.pipeline_dir = utils.fix_path( settings.pipeline_dir );
        console.log("Settings loaded.");
        console.log("Pipeline directory: " + settings.pipeline_dir);

    } catch( err ) {
        console.log("settings.json error: ");
        console.log( err );
        return -1;
    }
}
load_settings();




// Load worksession file
var worksession = undefined;
function load_default_worksession() {
    try {
        console.log("Loading default worksession file");
        worksession = require("./worksession.json");
        worksession.cam0_datadir = utils.fix_path( worksession.cam0_datadir );
        worksession.cam1_datadir = utils.fix_path( worksession.cam1_datadir );
        worksession.confdir = utils.fix_path( worksession.confdir );
        worksession.workdir = utils.fix_path( worksession.workdir );

    } catch( err ) {
        console.log("settings.json error: ");
        console.log( err );
        return -1;
    }
}
load_default_worksession();



var extexe_sanity_check = function( settings )
{
    var check_deferred = Q.defer();

    var t1 = Q.defer();
    var task = new RunTask.RunTask( settings.pipeline_dir+settings.prepare_exe, [], settings.pipeline_dir );
    task.start( function() { t1.resolve(); }, function(err) { t1.reject( new Error(err)); }  );

    var t2 = Q.defer();
    var task = new RunTask.RunTask( settings.pipeline_dir+settings.matcher_exe, [], settings.pipeline_dir );
    task.start( function() { t2.resolve(); }, function(err) { t2.reject( new Error(err)); }  );

    var t3 = Q.defer();
    var task = new RunTask.RunTask( settings.pipeline_dir+settings.autocalibrate_exe, [], settings.pipeline_dir );
    task.start( function() { t3.resolve(); }, function(err) { t3.reject( new Error(err)); }  );

    var t4 = Q.defer();
    var task = new RunTask.RunTask( settings.pipeline_dir+settings.stereo_exe, [], settings.pipeline_dir );
    task.start( function() { t4.resolve(); }, function(err) { t4.reject( new Error(err)); }  );

    Q.when(Q.all([t1.promise, t2.promise, t3.promise, t4.promise ]), function() {
        console.log("Pipeline executables looks good!");
        check_deferred.resolve();
    }, function(err) {
        console.log(">> " + err );
        check_deferred.reject();
    });

    return check_deferred.promise;
}




var load_wdirs = function( worksession ) {

    try {
        var filename = worksession.workdir+"workspaces.txt";
        var content = fs.readFileSync(filename)+"";
        var all_wdirs = content.split("\n");
        for( i=0; i<all_wdirs.length; ++i ) {
            if( (all_wdirs[i]).length == 0 )
                delete all_wdirs[i];
        }
        return all_wdirs;

    } catch(error) {
        console.log("Unable to load " + filename);
        console.log(error);
        return [];
    }

}


var prepare_data = function( worksession, mainq ) {

    var scandef = Q.defer();

    try {

        try {
            fs.accessSync( worksession.workdir , fs.F_OK );
        } catch(e) {
            fs.mkdirSync( worksession.workdir );
        }

        fs.accessSync( worksession.workdir , fs.F_OK );

    } catch( e ) {
        scandef.reject( worksession.workdir + " is not an accessible directory.");
        return scandef.promise;
    }

    worksession.currstatus = "Scanning cam0/cam1 data dir";

    setTimeout( function() {
        var datadir = new dirscan.DirScan(worksession.cam0_datadir, worksession.cam1_datadir, worksession.seq_start, worksession.seq_end, function( cam0, cam1) {

            console.log("Dir scan completed");

            worksession.wdir_frames = [];
            var image_indices = Object.keys( cam0 );
            if( image_indices.length < 1 )
                scandef.reject("No frames found.");

            image_indices.forEach( function(idx) {

                mainq.create( 'prepare', {
                    title:('['+sprintf("%06d_wd",idx)+"] Preparing data" ),
                    index: idx,
                    cam0file: cam0[idx],
                    cam1file: cam1[idx],
                    workspacename: sprintf("%06d_wd/",idx)
                }).save();

            });
            worksession.currstatus = "prepare";
            worksession.total_jobs = image_indices.length;
            scandef.resolve();

        }, function( err ) {
            scandef.reject(err);
        } );
    }, 10 );

    return scandef.promise;
}


var match = function( worksession, mainq ) {

    worksession.wdir_frames = load_wdirs(worksession);

    var wdir_subset_indices = [];
    var wdir_indices = Object.keys(worksession.wdir_frames);
    var nframes = settings.num_frames_to_match > wdir_indices.length ? wdir_indices.length : settings.num_frames_to_match;

    //randomize
    for(i=0;i<wdir_indices.length*2;++i) {
        var idx1 = Math.floor((Math.random()*(wdir_indices.length)));
        var idx2 = Math.floor((Math.random()*(wdir_indices.length)));
        var aux = wdir_indices[idx1];
        wdir_indices[idx1] = wdir_indices[idx2];
        wdir_indices[idx2] = aux;
    }
    for(i=0;i<nframes;++i)
        wdir_subset_indices.push( wdir_indices[i] );

    console.log("Using the following wdir_subset: " + wdir_subset_indices );

    //worksession.wdir_frames.forEach( function(wdir) {
    wdir_subset_indices.forEach( function(wdir_idx) {
        var wdir = worksession.wdir_frames[wdir_idx];
        if( wdir.length>0 ) {
            mainq.create( 'match', {
                title:('['+wdir+'] Matching' ),
                workspacename: wdir
            }).save();
        }
    });
    worksession.currstatus = "match";
    worksession.total_jobs = worksession.wdir_frames.length;

}

var load_extrinsics = function( worksession, mainq ) {

    worksession.wdir_frames = load_wdirs(worksession);

    var extfileR = worksession.confdir + "/ext_R.xml";
    if( !fs.existsSync(extfileR) ) {
        return extfileR + " not found.";
    }
    var extfileT = worksession.confdir + "/ext_T.xml";
    if( !fs.existsSync(extfileT) ) {
        return extfileT + " not found.";
    }

    worksession.wdir_frames.forEach( function(wdir) {
        if( wdir.length>0 ) {
            mainq.create( 'extload', {
                title:('['+wdir+'] Loading extrinsics' ),
                workspacename: wdir,
                extfileR: extfileR,
                extfileT: extfileT
            }).save();
      }
    });

    worksession.currstatus = "extload";
    worksession.total_jobs = worksession.wdir_frames.length;
    return;
}

var dense = function( worksession, mainq ) {

    worksession.wdir_frames = load_wdirs(worksession);
    worksession.wdir_frames.forEach( function(wdir) {
        if( wdir.length>0 ) {
            mainq.create( 'dense', {
                title:('['+wdir+'] Dense stereo' ),
                workspacename: wdir
            }).on('remove',function(result) {
                console.log("JOB REMOVED");
            }).on('complete',function(result) {
                console.log("JOB COMPLETE");
            }).save();
      }
    });
    worksession.currstatus = "dense";
    worksession.total_jobs = worksession.wdir_frames.length;
}


var matchmerge = function( worksession, mainq ) {

    mainq.create("matchmerge", {
        title:'Calibrating extrinsic parameters',
        workspacelistfile: worksession.workdir+"workspaces.txt"
    }).save();

    worksession.currstatus = "matchmerge";
    worksession.total_jobs = 1;
}

var save_worksession = function() {
    var wsdata = JSON.stringify(worksession);
    fs.writeFileSync( worksession.workdir + "/worksession.json", wsdata );
}


var load_worksession = function( workdir ) {

    var wsfile = workdir+"/worksession.json";
    console.log(wsfile);
    if( fs.existsSync( wsfile ) ) {
        console.log("Loading " + wsfile);
        var wsdata = fs.readFileSync( wsfile );
        try {
            var newws = JSON.parse(wsdata);
            if( newws.workdir==workdir ) {
                worksession = newws;
                console.log("Worksession successfully loaded!");
                return true;
            }
        } catch( error ) {
            console.log("JSON parse error: " + error );
        }
    }

    return false;
}


Q.when( extexe_sanity_check(settings), function() {

    console.log("Bootstrapping...");
    var dbootstrap = Q.defer();

    worksession.currstatus = "idle";
    worksession.total_jobs = 0;
    worksession.completed_tasks = {
        prepare: false,
        match: false,
        matchmerge: false,
        extload: false,
        dense: false
    }
    var start_time = Date.now();
    var elapsed = Date.now() - start_time;

    var mainq = kue.createQueue( kuesettings );
    var qlen = 0;
    var numfailures = 0;

    var reset_status = function() {
        worksession.currstatus = "idle";
        worksession.total_jobs = 0;
        start_time = Date.now();
        elapsed = 1;
        save_worksession();
    }
    var on_queue_empty = function() { console.log("Queue empty!"); reset_status(); };
    var on_enqueue = function(id,type) {
        qlen = qlen+1;
        console.log( '[ ] JOB  %s (%s) ', id, type );
    };
    var on_completed = function(id,type) {
        console.log( '[X] JOB  %s', id  );
        elapsed = Date.now()-start_time;
        qlen = qlen-1;
        if( qlen === 0 ) {
            worksession.completed_tasks[ worksession.currstatus ] = true;
            reset_status();
            on_queue_empty();
        }
    }
    var on_fail = function(id,err) {
        console.log( '[E] JOB %s: %s', id,err  );
        qlen = qlen-1;
        numfailures=numfailures+1;
        if( qlen === 0 ) {
            on_queue_empty();
        }
    }
    var get_full_status = function() {
        var progress_c =  worksession.total_jobs-qlen;
        var remaining = (elapsed*worksession.total_jobs/progress_c) - elapsed;
        return  {
            completed_tasks: worksession.completed_tasks,
            current_status: worksession.currstatus,
            progress: progress_c,
            progress_max: worksession.total_jobs,
            elapsed_time: elapsed,
            remaining: remaining,
            num_failed: numfailures
        };
    }

    var start_workers = function() {

        mainq = kue.createQueue( kuesettings );

        mainq.on('job enqueue', on_enqueue );
        mainq.on('job failed',on_fail );
        mainq.on('job complete',on_completed );

        /**
         * # Match job
         *
         *  Performs wass_match to recover
         *  rigid motion between each separate frames
         */
        mainq.process('match', settings.match_parallel_jobs, function(job,ctx,done) {


            var matchtask = new RunTask.RunTask( settings.pipeline_dir+settings.matcher_exe,
                                                [worksession.confdir+worksession.match_config_file,worksession.workdir+job.data.workspacename],
                                                worksession.workdir+job.data.workspacename,
                                                "matchlog.txt" );

            matchtask.start( function(code) {
                if( code!==0 ) {
                    return done( new Error('Bad return code') );
                }
                return done();
            }, function(err) {
                return done( new Error(err) );
            }, function(i,n) {
                job.progress(i,n);
            });

        });


        /**
         * # Dense reconstruction job
         *
         *  Performs wass_stereo on all the workspaces
         */
        mainq.process('dense', settings.stereo_parallel_jobs, function(job,ctx,done) {

            /*
            var redis = new ioRedis();

            redis.subscribe('workstartpause', function (err, count) {
                if( err!==null ) {
                    console.log("Unable to subscribe: %s",err);
                }
            });

            redis.on('message', function (channel, message) {
                console.log('Receive message %s from channel %s', message, channel);
                if( message === 'PAUSE' ) {
                    console.log("Pausing worker");
                    ctx.pause( 180000, function(err) {
                        if( err!==null ) {
                            console.log("Pause error: %s", err );
                        }
                        console.log("Paused!");
                    });
                }
                if( message === 'RESUME' ) {
                    console.log("Resuming worker");
                    ctx.resume();
                }
            });
            /**/
            /*
            ctx.pause( 50000, function(err){
                console.log("Worker is paused... ");
                setTimeout( function(){ ctx.resume(); }, 5000 );
            });
            /**/

            var workspacepath = worksession.workdir+job.data.workspacename;
            var workspace_num = parseInt( job.data.workspacename.slice(0,6));
            var config_file = worksession.confdir + worksession.dense_stereo_config_file;
            var task = new RunTask.RunTask( settings.pipeline_dir+settings.stereo_exe,
                                           [config_file, workspacepath],
                                           workspacepath,
                                           "densestereolog.txt" );


            task.start( function(code) {
                if( code!==0 ) {
                    console.log("DONE");
                    return done( new Error("Bad return code, check log") );
                }

                var pl = fs.readFileSync( workspacepath+'plane.txt', 'utf-8' ).replace(/\n/g,' ').replace(/\r/g,' ') ;
                fs.appendFileSync( worksession.workdir+'/planes.txt', pl+'\n' );

                // Delete source images if necessary
                if( !worksession.keepimages && workspace_num > worksession.keep_images_end ) {
                    utils.deleteRecursive( workspacepath+"undistorted" );
                }

                // Cleanup some disk space
                if( worksession.savediskspace ) {
                    utils.deleteRecursive( workspacepath+"disparity_coverage.jpg" );
                    utils.deleteRecursive( workspacepath+"disparity_final_scaled.png" );
                    utils.deleteRecursive( workspacepath+"stereo_input.jpg" );
                    utils.deleteRecursive( workspacepath+"stereo.jpg" );
                    utils.deleteRecursive( workspacepath+"plane_refinement_inliers.xyz" );
                    utils.deleteRecursive( workspacepath+"mesh.ply" );
                    utils.deleteRecursive( workspacepath+"mesh_full.ply" );
                }

                if( process.platform != 'win32' && worksession.zipoutput ) {
                    var full_workspace_name = job.data.workspacename.slice(0,9);

                    var tartask = new RunTask.RunTask( "zip",
                        ["-9", "-r", full_workspace_name+".zip", job.data.workspacename],
                        workspacepath+"../" );

                    tartask.start( function(code) {
                        utils.deleteRecursive( workspacepath );
                        console.log("DONE");
                        return done();
                    }, function(err) {
                        console.log("DONE");
                        done( new Error("ZIP error") );
                    });

                }

                console.log("DONE");
                return done();

            }, function(err) {
                console.log("DONE");
                done( new Error(err) );
            }, function(i,n) {
                job.progress(i,n);
            });
        });


        /**
         * # Load extrinsic calibration job
         *
         */
        mainq.process('extload', 1, function(job,ctx,done) {

            var workspacepath = worksession.workdir+job.data.workspacename;
            fs.writeFileSync( workspacepath+"/ext_R.xml",
                             fs.readFileSync(job.data.extfileR));
            fs.writeFileSync( workspacepath+"/ext_T.xml",
                             fs.readFileSync(job.data.extfileT));

            return done();
        });


        /**
         * # MatchMerge job
         *
         *  Performs wass_autocalibrate on all the workspaces
         *  to calibrate extrinsic camera parameters
         */
        mainq.process('matchmerge', 1, function(job,ctx,done) {

            var task = new RunTask.RunTask( settings.pipeline_dir+settings.autocalibrate_exe,
                                            [job.data.workspacelistfile],
                                            worksession.workdir,
                                            "autocalibrate.txt" );

            task.start( function(code) {
                if( code!==0 ) {
                    return done( new Error("Bad return code, check log") );
                }
                return done();

            }, function(err) {
                done( new Error(err) );
            }, function(i,n) {
                job.progress(i,n);
            }
                      );
        });


        /**
         * # Prepare job
         *
         *  Performs wass_prepare on the frames specified in
         *  job.data.cam0file, job.data.cam1file
         */
        mainq.process('prepare', settings.prepare_parallel_jobs, function(job,ctx,done) {

            var workspacepath = worksession.workdir+job.data.workspacename;
            var logfile = sprintf("%06d.log", job.data.index);
            var args = [
                "--workdir", workspacepath,
                "--calibdir", worksession.confdir,
                "--c0", worksession.cam0_datadir+job.data.cam0file,
                "--c1", worksession.cam1_datadir+job.data.cam1file
            ];

            // prepare the workspaces.txt file
            var task = new RunTask.RunTask( settings.pipeline_dir+settings.prepare_exe,
                                            args,
                                            worksession.workdir,
                                            logfile );

            task.start( function(code) {
                if( code!==0 ) {
                    return done( new Error("Bad return code, check log") );
                }

                fs.unlinkSync( worksession.workdir+logfile );
                worksession.wdir_frames.push( job.data.workspacename );

                fs.appendFileSync( worksession.workdir+"/workspaces.txt", job.data.workspacename+"\n" );

                return done();

            }, function(err) {
                done( new Error(err) );

            }, function(i,n) {
                job.progress(i,n);
            }
                      );

        });

    }


    var dstart = Q.defer();
    var init_tasks = [ dstart.promise ];
    ['prepare','match','matchmerge','extload','dense'].forEach( function( jobname ) {
        init_tasks.push( utils.deactivate_all_active( jobname ) );
        init_tasks.push( utils.remove_all_completed( jobname ) );
        init_tasks.push( utils.remove_all_failed( jobname ) );
        dstart.resolve();
    });
    //dstart.resolve();

    Q.when( Q.all( init_tasks ) , function() {

        console.log("Queue initialization completed, starting workers");

        start_workers()

        // Start kue app to show progress
        var port = settings.http_port || 3000;
        kue.app.listen(port);
        console.log("----------------------------------------------------");
        console.log("   Progress monitor available at: ");
        console.log("     http://localhost:%s",port);
        console.log("");
        console.log("   WassMonitor: ");
        console.log("     http://localhost:8080");
        console.log("----------------------------------------------------");

        load_worksession( worksession.workdir );

        dbootstrap.resolve();
    });


    Q.when( dbootstrap.promise, function() {
        console.log("Bootstrap completed.");


        /*
        setInterval( function() {

            // Manually remove each completed jobs so that we can
            // better track jobs completion even with faulty kue events
            //
            kue.Job.rangeByState( 'complete', 0, 10, 'asc', function( err, jobs ) {
                console.log( JSON.stringify(jobs) );
                if( !err )
                    jobs.forEach( function( job ) {
                        on_completed( job.id, null);
                        job.remove( function(){
                            //console.log( 'removed ', job.id );
                        });
                    });
            });
        },1000);
        */

        //prepare_data( worksession, mainq );
        //match(worksession,mainq);
        //matchmerge(worksession,mainq);
        //dense( worksession,mainq );
        //
        //
        var http = require('http');
        var server = new http.Server();
        server.listen( 8080 );
        server.on("request", function(request,response) {
            // Parse the requested URL
            var url = require('url').parse(request.url);
            var not_idle_state_error = function() {
                response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                response.write( JSON.stringify( {
                    success: false,
                    reason: "Not in idle state"
                } ) );
                response.end();
            };

            if(url.pathname === "/doprepare") {
                if( worksession.currstatus == "idle" ) {
                    /*
                    prepare_data(worksession,mainq);
                    response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                    response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                    response.write( JSON.stringify( {
                        success: true,
                        reason: ""
                    } ) );
                    response.end();
                    */
                    Q.when( prepare_data(worksession,mainq), function() {
                        response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                        response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                        response.write( JSON.stringify( {
                            success: true,
                            reason: ""
                        } ) );
                        response.end();
                    }, function(err) {
                        response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                        response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                        response.write( JSON.stringify( {
                            success: false,
                            reason: err
                        } ) );
                        response.end();
                    });
                    return;
                } else {
                    not_idle_state_error();
                    return;
                }
            }
            else if(url.pathname === "/domatch") {
                if( worksession.currstatus == "idle" ) {
                    match(worksession,mainq);
                    response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                    response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                    response.write( JSON.stringify( {
                        success: true,
                        reason: ""
                    } ) );
                    response.end();
                    return;
                } else {
                    not_idle_state_error();
                    return;
                }
            }
            else if(url.pathname === "/domatchmerge") {
                if( worksession.currstatus == "idle" ) {
                    matchmerge(worksession,mainq);
                    response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                    response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                    response.write( JSON.stringify( {
                        success: true,
                        reason: ""
                    } ) );
                    response.end();
                    return;
                } else {
                    not_idle_state_error();
                    return;
                }
            }
            else if(url.pathname === "/doextload") {
                if( worksession.currstatus == "idle" ) {
                    var err = load_extrinsics(worksession,mainq);

                    if( err ) {
                        response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                        response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                        response.write( JSON.stringify( {
                            success: false,
                            reason: err
                        } ) );
                        response.end();
                   } else {
                        response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                        response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                        response.write( JSON.stringify( {
                            success: true,
                            reason: ""
                        } ) );
                        response.end();
                    }
                    return;
                } else {
                    not_idle_state_error();
                    return;
                }
            }
            else if(url.pathname === "/dodense") {
                if( worksession.currstatus == "idle" ) {
                    dense(worksession,mainq);
                    response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                    response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                    response.write( JSON.stringify( {
                        success: true,
                        reason: ""
                    } ) );
                    response.end();
                    return;
                } else {
                    not_idle_state_error();
                    return;
                }
            }
            else if(url.pathname === "/fullstatus") {
                response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                response.write( JSON.stringify( get_full_status() ) );
                response.end();
                return;
            }
            else if(url.pathname === "/worksession") {
                response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                response.write( JSON.stringify( worksession ) );
                response.end();
                return;
            }
            else if( url.pathname === "/reloadworksession" ) {

                response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                response.writeHead(200, {"Access-Control-Allow-Origin": "*"});

                var opstatus = { error: false, msg: ""};

                if( worksession.currstatus == "idle" ) {
                    load_default_worksession();
                    opstatus.error = !load_worksession( worksession.workdir );
                    opstatus.curr_worksession = worksession;
                }

                response.write( JSON.stringify( opstatus ) );
                response.end();
                return;
            }
            else if( url.pathname === "/pausequeue" ) {

                response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                response.writeHead(200, {"Access-Control-Allow-Origin": "*"});

                var opstatus = { error: false, msg: ""};

                var redis = new ioRedis();
                redis.publish('workstartpause', 'PAUSE');


                response.write( JSON.stringify( opstatus ) );
                response.end();
                return;
            }
            else if( url.pathname === "/resumequeue" ) {

                response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                response.writeHead(200, {"Access-Control-Allow-Origin": "*"});

                var opstatus = { error: false, msg: ""};

                var redis = new ioRedis();
                redis.publish('workstartpause', 'RESUME');

                response.write( JSON.stringify( opstatus ) );
                response.end();
                return;
            }
            else if(url.pathname === "/activejobs") {
                Q.when(utils.get_active_jobs(worksession.currstatus), function (value) {
                    response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                    response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                    response.write( JSON.stringify( value ) );
                    response.end();
                }, function (error) {
                    response.writeHead(200, {"Content-Type": "application/json; charset=UTF-8"});
                    response.writeHead(200, {"Access-Control-Allow-Origin": "*"});
                    response.write( JSON.stringify( [] ) );
                    response.end();
                });
                return;
            } else {
                // Get local filename and guess its content type based on its extension.
                var resname = url.pathname.substring(1);
                if( resname.length==0 )
                    resname = "index.html";
                var filename = "WassMonitor/"+resname; // strip leading /
                var type;
                switch(filename.substring(filename.lastIndexOf(".")+1)) { // extension
                    case "html":
                        case "htm": type = "text/html; charset=UTF-8"; break;
                    case "js": type = "application/javascript; charset=UTF-8"; break;
                    case "css":type = "text/css; charset=UTF-8"; break;
                    case "woff":type = "application/font-woff"; break;
                    case "woff2":type = "application/font-woff2"; break;
                    case "svg":type = "image/svg+xml"; break;
                    default: type = "application/octet-stream"; break;
                }

                // Read the file
                // chunk to the callback function. For really large files, using the
                //  streaming API with fs.createReadStream() would be better.
                fs.readFile(filename, function(err, content) {
                    //console.log("------- General request");
                    //console.log("Res: "+resname);
                    //console.log("File: " + filename);
                    //console.log("Type: " + type );
                    if (err) { // If we couldn't read the file for some reason
                        response.writeHead(404, { // Send a 404 Not Found status
                            "Content-Type": "text/plain; charset=UTF-8"});
                            response.write(err.message);
                            console.log("Resource not found: " + filename);
                            response.end();
                    }
                    else {
                        response.writeHead(200, {"Content-Type": type});
                        response.write(content);
                        response.end();
                        //console.log("Resource served!");
                    }
                    //console.log("-------");
                });
            }

        });
    });

});


