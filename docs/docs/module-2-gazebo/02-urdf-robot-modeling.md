---
sidebar_position: 2
---

# Chapter 2: URDF Robot Modeling

Now that you have a virtual world to play in, you need a robot to put in it. In ROS 2, the standard way to describe a robot's physical structure is with the **Unified Robot Description Format (URDF)**. This chapter will teach you how to create a simple robot model using URDF.

## What is URDF?

URDF is an XML-based file format used in ROS to describe the physical characteristics of a robot. It's like a blueprint for your robot's digital twin. A URDF file describes the robot as a tree of rigid bodies, called **links**, connected by **joints**.

With a URDF file, you can:
*   **Visualize your robot:** See what your robot looks like in 3D using tools like RViz.
*   **Simulate your robot:** Use simulators like Gazebo to test your robot's behavior in a virtual environment.
*   **Perform kinematic and dynamic calculations:** Use ROS libraries to calculate the robot's motion and forces.

## The Core Components of URDF

A URDF file has two main components: `<link>` and `<joint>`.

### Links

A `<link>` element describes a rigid part of the robot. Think of it as a single, solid piece of your robot's structure. Each link has a unique name and can have three key properties:

*   **`<visual>`:** Describes how the link looks. This includes its shape (like a box, cylinder, or a 3D mesh), its color, and its position and orientation relative to the link's origin.
*   **`<collision>`:** Describes the link's physical shape for collision detection. This is often a simplified version of the visual geometry to improve simulation performance.
*   **`<inertial>`:** Defines the link's physical properties, such as its mass and inertia. These are crucial for realistic physics simulation.

### Joints

A `<joint>` element connects two links and defines how they can move relative to each other. Each joint has a unique name and a type, which determines the kind of motion allowed. Common joint types include:

*   **`revolute`:** A rotating joint with a limited range of motion (like an elbow).
*   **`continuous`:** A rotating joint with no limits (like a wheel).
*   **`prismatic`:** A sliding joint with a limited linear range.
*   **`fixed`:** A rigid connection between two links, with no relative motion. This is useful for attaching sensors or other components that don't move.

Each joint also specifies a **parent** link and a **child** link, forming the tree structure of the robot.

## Creating a Simple Robot URDF

Let's create a URDF for a very simple robot: a base with a single rotating arm.

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- A simple blue box for the base -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- A red cylinder for the arm -->
  <link name="arm_link">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- The joint connecting the base and the arm -->
  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

</robot>
```

### Breaking Down the URDF

*   We define two links: `base_link` and `arm_link`.
*   Each link has `<visual>`, `<collision>`, and `<inertial>` properties.
*   We define one joint: `base_to_arm_joint`.
    *   It's a `revolute` (rotating) joint.
    *   It connects `base_link` (the parent) to `arm_link` (the child).
    *   The `<origin>` tag places the arm on top of the base.
    *   The `<axis>` tag specifies that the arm rotates around the Z-axis.
    *   The `<limit>` tag constrains the joint's rotation to +/- 90 degrees.

## Spawning Your Robot in Gazebo

Now that you have a URDF file, how do you get it into Gazebo?

Gazebo natively uses the **Simulation Description Format (SDF)**, which is a superset of URDF. Fortunately, Gazebo can automatically convert URDF to SDF.

The easiest way to spawn a URDF model in Gazebo is to use a ROS 2 service call.

1.  **Save your URDF:** Save the code above as `simple_robot.urdf`.
2.  **Start Gazebo:** Launch an empty world as you did in the previous chapter:
    ```bash
    gazebo --verbose /opt/ros/humble/share/gazebo_plugins/worlds/empty.world
    ```
3.  **Spawn the model:** In a new terminal, use the `gz` command-line tool to call the `/world/empty/create` service. Make sure to replace `/path/to/your/file/` with the actual absolute path to your `simple_robot.urdf` file.
    ```bash
    gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/your/file/simple_robot.urdf", name: "simple_robot"'
    ```

If successful, you should see your simple robot appear in the Gazebo world!

## What's Next?

You've learned how to describe a robot's structure with URDF and spawn it into a Gazebo simulation. However, our robot is just a static model. To make it do something, we need to add sensors and actuators.

In the next chapter, you will learn how to add Gazebo-specific tags to your URDF to simulate physics and add sensors, bringing your robot one step closer to life.
