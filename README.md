

<p align="center">
  <a href="https://pkg.go.dev/github.com/erh/vmodutils"><img src="https://pkg.go.dev/badge/github.com/erh/vmodutils" alt="PkgGoDev"></a>
</a>
</p>

## pc crop camera
```
{
  "src" : "<cam>",
  "src_frame" : <optional>, // src point cloud will be converted to world from this, if not specified assume it is world
  "min" : { "X" : 0, "Y" : 0, "Z" : 0}, // specified in world frame
  "min" : { "X" : 9, "Y" : 9, "Z" : 9}  // specified in world frame
}
  
```

## pc detect crop camera
```
{
  "src" : "<cam>",
  "service" : "<vision service>"
}
```

## pc merge
```
{
  "cameras" : ["<cam>"]
}
```

## arm position saver
```
{
    // required
    "arm" : "<name of arm>",

    // optional
    // if motion and joints are set, uses joint to joint motion via motion.Move
    // if motion and pose are set, uses cartesian motion via motion.Move
    // if motion is not set and joints are set, uses arm.MoveToJointPositions
    "motion" : "<name of motion service>",

    // can be set automatically via SetPosition command
    "joints" : [ ],
    "point" : < ... >,
    "orientation" : < ... >,

    // optional - if set, the Geometry objects in the vision services' GetObjectPointClouds results
    // are added to the world state passed to the motion service
    "vision_services": ["<name of vision service>"],

    // optional - options passed as 'extra' to motion.Move or arm.MoveToJointPositions
    "extra" : "<options>"
}
```

## multi arm position switch
```
{
    // required
    "arm" : "<name of arm>",

    // optional - if set uses joint to joint motion via motion.Move, if not uses arm.MoveToJointPositions
    "motion" : "<name of motion service>",

    // list of arm joint positions
    // the list needs to contain at least one set of joint positions
    "joints" : [[0, 0, 0, 0, 0, 0], ...],

    // optional - if set, the Geometry objects in the vision services' GetObjectPointClouds results
    // are added to the world state passed to the motion service
    "vision_services": ["<name of vision service>"],

    // optional - options passed as 'extra' to motion.Move or arm.MoveToJointPositions
    "extra" : "<options>"
}
```

## pc multiple arm poses
```
{
 "src" : "<name of camera>",
 "positions" : [ <arm-position-saver>, ... ]
 }
```

## obstacle
Configure this with a frame and you can have obstacles on your robot without having to hard code.
```
{
 "geometries" : [ { "type" : "box", "x" : 100, "y": 100, "z" : 100 } ]
 "geometries" : [ { "type" : "sphere", "r" : 100 } ]

}
```

## obstacle open box
Configure this with a frame and you can have obstacles on your robot without having to hard code.
```
{
  "length" : 10,
  "width" : 10,
  "height" : 10,
  "thickness" : <optional, defaults to 1>,
  "to_move" : <if you want to move something to grab>,
  "motion" : <needed it to_move, but will also default to builtin>
}
```

## pc look at crop camera
looks at the center of a point cloud and gets just that
```
{
    "src" : "<camera>",
    "use_color" : "<bool>" // optional
}
```
