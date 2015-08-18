positionStream = new Meteor.Stream('position');

if (Meteor.isClient) {
  Meteor.call("logToConsole", "Client working");

  positionStream.on('update', function(oldx, oldy, newx, newy) {
    drawPixi(oldx, oldy, newx, newy);
  });
}

if (Meteor.isServer) {
  Fiber = Npm.require('fibers');
  oldx = -1;
  oldy = -1;
  
  console.log('Server working');
  
  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server3.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  Meteor.methods({
    logToConsole: function(msg) {
      console.log(msg);
    }
  });
    
  // Subscribing to a Topic
  // ----------------------
  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/turtle1/pose',
    messageType : 'turtlesim/Pose'
  });

  listener.subscribe(function(message) {
    newx = message.x;
    newy = message.y;
    if (oldx == -1) {
      oldx = newx;
      oldy = newy;
      console.log('First movement - Initial position: (' + oldx + ', ' + oldy + ')');
    } else if (oldx != newx || oldy != newy) {
      //console.log('Draw: oldx = ' + oldx + ', oldy = ' + oldy + ', newx = ' + newx + ', newy = ' + newy);
      Fiber(function() {
        positionStream.emit('update', oldx, oldy, newx, newy);
        oldx = newx;
        oldy = newy;
      }).run();
    }
  });
}
