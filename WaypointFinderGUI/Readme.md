## Controls:

    [ -> Zoom out
    ] -> Zoom in
    Arrow Keys -> Pan the image
    r -> reset the view

    e -> select "ego" mode.
            To specify the waypoint to spawn the ego vehicle at
    o -> select "npc" (obstacle vehicles) mode
            Adds the waypoints clicked for npc spawning
    w -> select "waypoints" mode
            Adds the waypoints clicked for waypoint following for ego vehicle
    q -> comes back to default mode. Points clicked will not be saved

    s -> To save the edited file.

## Selecting points
Clicked points can be viewed in the main Carla window. Clicked points are drawn with a '**o**'. Closest waypoint is drawn with a '**x**'.

## JSON File
According to the selected modes, the data is saved in the *scene_x.json* file. **Set path to the folder** in the main() function. The file names are automatically handled.