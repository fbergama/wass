
(function(global)
 {
     var cp = require('child_process');
     var fs = require('fs');

     function RunTask( name, args, cwd, logfile ) 
     {
         this.process = undefined;
         this.args = args;
         this.name = name;
         this.cwd = cwd || './';
         this.logfile = logfile;
     }


     RunTask.prototype.start = function( onexit, onerror, progress ) 
     {
         var ostream;

         if( this.logfile ) {
             ostream = fs.createWriteStream( this.cwd+"/"+this.logfile );
         } else { 
             ostream = {
                 write: function(data) {},
                 end: function(){}
             }
         }

         var procname = this.name;
//       console.log(procname +  " " + this.args );
         this.process = cp.spawn(procname, this.args, {
             cwd: this.cwd,
         } ).on("error",function(err){ onerror('Unable to spawn child process '+procname); });
         
         this.process.stdout.on("data", function(data) {

//           console.log(data+"");

             // Check if we can match a progress string
             var stringdata = data.toString();
             var mts = stringdata.match( /\[P\|(\d*)\|(\d*)\]/ );

             // If match was ok report progress to the caller
             if( progress && mts && mts.length>2 && !isNaN(parseFloat(mts[1])) &&  !isNaN(parseFloat(mts[2]))) {
                 progress( mts[1], mts[2]);
             }

             ostream.write(data);

         });

         this.process.stderr.on("data", function(data) {
             console.log(data+"");
             ostream.write(data);
         });

         this.process.on("close", function(code) {
             ostream.end();
//           console.log("Process terminated");

             if( typeof(onexit)==='function')
                 onexit(code);

             this.process = undefined;
         });
     }


     module.exports.RunTask = RunTask;

 })(this);
