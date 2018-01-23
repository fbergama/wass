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


(function(global){

    var Q = require('q');
    var kue = require('kue');

    module.exports.fix_path = function( name ) {

        var path = require('path');
        if( !path.isAbsolute(name) )
            name = path.resolve( name );

        if( name[ name.length - 1] !== '/' )
            name = name + "/";

        return name;
    }

    /**
     * # Deletes a file or a directory in a recursive way
     * (ie. if path is a directory, deleteRecursive is called
     * on each contained element
     */
    module.exports.deleteRecursive = function(path) {
        var fs = require('fs');
        if( fs.existsSync(path) ) {

            if( fs.lstatSync( path ).isDirectory() ) {
                fs.readdirSync(path).forEach(function(file,index){
                    var curPath = path + "/" + file;
                    module.exports.deleteRecursive(curPath);
                });
                fs.rmdirSync(path);
            } else {
                fs.unlinkSync(path);
            }
        }
    };

    /**
     * # Set all active <jobname> jobs to inactive
     */
    module.exports.deactivate_all_active = function( jobname ) {
        var def = Q.defer();

        require('kue').Job.rangeByType(jobname, 'active', 0, -1, 'asc', function (err, selectedJobs) {
            console.log("Reverting %d old (%s) active jobs to inactive", selectedJobs.length, jobname);
            selectedJobs.forEach(function (job) {
                job.remove();//temp
            });
            def.resolve();
        });

        return def.promise;
    }


    /**
     * # Returns all the active jobs
     */
    module.exports.get_active_jobs = function( jobname ) {
        var deferred = Q.defer();
        require('kue').Job.rangeByType(jobname, 'active', 0, -1, 'asc', function (err, selectedJobs) {
            deferred.resolve( selectedJobs );
        });
        return deferred.promise;
    }


    /**
     * # Removes all the completed <jobname> jobs
     */
    module.exports.remove_all_completed = function( jobname ) {
        var def = Q.defer();

        require('kue').Job.rangeByType( jobname, 'complete', 0, -1, 'asc', function (err, selectedJobs) {
            console.log("Purging %d old (%s) completed jobs", selectedJobs.length, jobname );
            selectedJobs.forEach(function (job) {
                job.remove();
            });
            def.resolve();
        });

        return def.promise;
    }


    /**
     * # Removes all the failed <jobname> jobs
     */
    module.exports.remove_all_failed = function( jobname ) {

        var def = Q.defer();

        require('kue').Job.rangeByType( jobname, 'failed', 0, -1, 'asc', function (err, selectedJobs) {
            console.log("Purging %d old (%s) failed jobs", selectedJobs.length, jobname );
            selectedJobs.forEach(function (job) {
                job.remove();
            });
            def.resolve();
        });

        return def.promise;
    }

})(this);
