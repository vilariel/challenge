Package.describe({
  name: 'ros',
  summary: 'ROS integration via roslibjs',
  version: '0.0.1',
});

Npm.depends({
 "eventemitter2": "0.4.14",
 "xmlserializer": "0.3.3",
 "xmldom": "0.1.19",
 "ws": "0.7.2"
});

Package.onUse(function(api) {
  api.addFiles('roslib.js', 'server');
  api.export('ROSLIB', 'server');
  api.export('Topic', 'server');
  api.export('Message', 'server');
  api.export('Ros', 'server');
});
