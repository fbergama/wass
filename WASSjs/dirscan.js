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


(function(global)
 {
     var Q = require('q');
     var fs = require('fs');

     var IMG_REGEX = /^(\d*)_*/;


     function filter_dir( files, out_files, seq_start, seq_end )
     {
         if( out_files == undefined )
             return;

         for( i=0; i<files.length; ++i ) {
             var mts = files[i].match( IMG_REGEX );
        
             if( !isNaN(parseFloat(mts[1])) && mts[1]<=seq_end && mts[1]>=seq_start ) {
                 out_files[ mts[1] ] = files[i];
             }
         }
     }


     function filter_intersection( files_cam0, files_cam1 ) {
         var maxlen = files_cam0.length > files_cam1.length ? files_cam0.length : files_cam1.length;

         for( i=0; i<maxlen; ++i ) {
             var f0 = files_cam0[i];
             var f1 = files_cam1[i];
            if( f0  == undefined ) {
                delete files_cam1[i];
            }
            if( f1 == undefined ) {
                delete files_cam0[i];
            }
         }
     }


     function DirScan( cam0_dir, cam1_dir, seq_start, seq_end, on_scan_completed, on_scan_error  )
     {
         console.log("DirScan");

         var cam0def = Q.defer();
         var cam1def = Q.defer();
         var myself=this;
         
         this.cam0images = [];
         this.cam1images = [];

         fs.readdir( cam0_dir, function(err,files) {
             if( err ) {
                return cam0def.reject("Unable to scan " + cam0_dir );
             }
             filter_dir( files, myself.cam0images, seq_start, seq_end ); 
             cam0def.resolve();
         } );

         fs.readdir( cam1_dir, function(err,files) {
             if( err ) {
                return cam1def.reject("Unable to scan " + cam1_dir );
             }
             filter_dir( files, myself.cam1images, seq_start, seq_end ); 
             cam1def.resolve();
         } );


         Q.when(Q.all([cam0def.promise, cam1def.promise]), function() {

             filter_intersection( myself.cam0images, myself.cam1images );

             /**/
             console.log("Scan completed.");
             console.log("Cam0 files:");
             console.log(myself.cam0images);
             console.log("Cam1 files:");
             console.log(myself.cam1images);
             /**/

             on_scan_completed( myself.cam0images, myself.cam1images );
             
         }, function(err) {
             if( on_scan_error )
                 on_scan_error( err );
         });
     }

     module.exports.DirScan = DirScan;

 })(this);
