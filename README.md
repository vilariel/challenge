# Challenge - Creativa77 - Ariel Vila

## Request

Create a [ROS](http://www.ros.org/) node that makes [turtlesim](http://wiki.ros.org/turtlesim) draw a star. During this process, whatever turtlesim draws has to be replicated by a [MeteorJs](https://www.meteor.com/) web application using [WebGL](https://en.wikipedia.org/wiki/WebGL). This means that every web browser viewing that application must show the same figure being drawn in (soft) real time.

The ROS node and the meteor server should work on an Ubuntu 14.04 box.

## Prerequisites

* Ubuntu 14.04 box

* Have ROS (Jade) base package installed. See installation procedure [here](http://wiki.ros.org/ROS/Installation)

* Have Meteor installed. See installation procedure [here](https://www.meteor.com/install)

* Have Npm installed
```sh
$ sudo apt-get install npm
```

* Have Git installed
```sh
$ sudo apt-get install git
```

* Have additional ROS packages installed: angles, turtlesim, rosbridge-server
```sh
$ sudo apt-get install ros-jade-angles
$ sudo apt-get install ros-jade-turtlesim
$ sudo apt-get install ros-jade-rosbridge-server
```

## Installation

```sh
$ git clone https://github.com/vilariel/challenge <yourpackagefolder>
```

Finish Drawer App installation
```sh
$ cd <yourpackagefolder>
$ aux/install_drawer_app.sh
```

Compile Drawer Node
```sh
$ cd <yourpackagefolder>/drawernode
$ catkin_make
```

## Tests

Start ROS core process
```sh
$ roscore
```

Launch the rosbridge server (from a new terminal)
```sh
$ roslaunch rosbridge_server rosbridge_websocket.launch
```

Launch the turtlesim (from a new terminal)
```sh
$ rosrun turtlesim turtlesim_node
```

Launch the Drawer Node (from a new terminal)
```sh
$ cd <yourpackagefolder>/drawernode
$ source devel/setup.bash
$ rosrun turtle_star star_drawer
```

Launch the Drawer App (from a new terminal)
```sh
$ cd <yourpackagefolder>/drawerapp
$ meteor
```

Open Drawer App in browser
```
http://localhost:3000
```

Draw Star (from a new terminal)
```sh
$ rostopic pub -1 /draw std_msgs/String '9 2'
```

Drawer Node is suscribed to /draw Topics which are of type std_msgs/String, consisting of two numbers: ‘edges radius’
- edges: corresponds to the number of points of the star to be drawn. This number must be an odd integer so the star can be drawn with a single stroke.
- radius: corresponds to the radius of the star to be drawn. This number should be 2.5 or smaller so the star fits the window at its starting point.

## Expected Result

Every browser pointing to [http://localhost:3000](http://localhost:3000) should display the same drawing made by the turtle in real time.

![screenshot](https://raw.githubusercontent.com/vilariel/challenge/master/aux/screenshot.png)

## API Reference

- [Roslibjs](http://wiki.ros.org/roslibjs) - Allow JavaScript to communicate with ROS over rosbridge
- [Meteor Streams](http://arunoda.github.io/meteor-streams/) - Realtime Messaging for Meteor
- [Pixijs](https://atmospherejs.com/fds/pixijs) - 2D webGL renderer with canvas fallback

