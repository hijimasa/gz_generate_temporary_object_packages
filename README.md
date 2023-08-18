# gz_generate_temporary_object_packages

## About
These are packages for Gazebo to create temporary objects that exist for a certain amount of time.

## Packages

- flying_disc_description
  This contains a target object URDF.

- ros_gz_sim_extra
  This contains "remove" command to remove Gazebo entities.
  "remove" need "-id" option to detect a target entity.
  The way to get a target entity's ID is written in "flying_disc_spawner".

- flying_disc_spawner
  This generates temporary objects by a topic (default "spawn_disc_pose").
  The position of the generated object is the position shifted by msg.pose based on the position of the entity whose name is written in msg.header.frame_id.
  The generated object is removed after "REMAIN_TIME_SEC" defined in "flying_disc_spawner/flying_disc_spawner/spawner.py".
