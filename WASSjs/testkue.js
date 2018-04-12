

function testkue( concurrency ) {

    console.log("Testing concurrency=%d", concurrency);

    var kue = require('kue');
    var queue = kue.createQueue();

    queue.on('job enqueue', function(id, type){
        console.log( 'job %s queued', id );

    }).on('job complete', function(id, result){
        console.log('job #%d completed', id);
    });


    queue.process('testtask', concurrency, function(job, ctx, done){

        ctx.pause( 5000, function(err){
            console.log("Worker is paused... ");
            setTimeout( function(){ ctx.resume(); console.log("Worker resumed"); }, 5000 );
        });
        setTimeout( function() { done(); }, 3000 );

    });


    for( var i=0; i<5; ++i ) {
        queue.create( 'testtask', {} ).save();
    }
}


testkue( 2 );

