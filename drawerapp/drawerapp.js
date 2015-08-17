positionStream = new Meteor.Stream('position');

if (Meteor.isClient) {
  Meteor.call("logToConsole", "Client working");

  positionStream.on('update', function(oldx, oldy, newx, newy) {
    drawPixi(oldx, oldy, newx, newy);
  });
  
  Template.actions.events({
    'click .turtle': function () {
      Meteor.call("moveTurtle");
    }
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
    },
    moveTurtle: function() {
      var cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/turtle1/cmd_vel',
        messageType : 'geometry_msgs/Twist'
      });
      var twist = new ROSLIB.Message({
        linear : {
          x : 3.0,
          y : 0.0,
          z : 0.0
        },
        angular : {
          x : 0.0,
          y : 0.0,
          z : 1.5
        }
      });
      cmdVel.publish(twist);
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
    //console.log('Received message on ' + listener.name + ': (' + message.x + ', ' + message.y + ')');
    newx = message.x;
    newy = message.y;
    if (oldx == -1) {
      console.log('First movement - Initial position');
      oldx = newx;
      oldy = newy;
    } else if (oldx != newx && oldy != newy) {
      console.log('Oldx = ' + oldx + ', oldy = ' + oldy + ', newx = ' + newx + ', newy = ' + newy);
      Fiber(function() {
        positionStream.emit('update', oldx, oldy, newx, newy);
        oldx = newx;
        oldy = newy;
      }).run();
    }
  });
}
