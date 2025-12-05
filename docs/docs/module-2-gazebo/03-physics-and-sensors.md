---
sidebar_position: 3
---

# Chapter 3: Physics and Sensors

Your robot model exists in the virtual world, but it doesn't yet interact with it in a meaningful way. It has no sense of physics and no way to perceive its environment. In this chapter, you'll learn how to add physics properties and sensors to your URDF, bringing your robot to life.

## The `<gazebo>` Element

As we've discussed, URDF is a general format for describing a robot's structure. Gazebo, however, has many advanced features that are not part of the standard URDF specification, such as physics properties, sensors, and plugins.

To add these Gazebo-specific properties to your robot, you use the `<gazebo>` element within your URDF file. This element allows you to specify things that only Gazebo will understand.

A common use of the `<gazebo>` element is to reference a link and add properties to it:
```xml
<gazebo reference="link_name">
  <!-- Gazebo-specific properties for this link go here -->
</gazebo>
```

## Adding Physics Properties

Let's make our simple robot from the previous chapter interact with Gazebo's physics engine. We'll add friction to the base so it doesn't slide around, and we'll add damping to the arm joint to simulate resistance.

Here's the updated `simple_robot.urdf` with physics properties:

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

  <!-- Add physics properties to the base_link -->
  <gazebo reference="base_link">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <material>Gazebo/Blue</material>
  </gazebo>

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
    <dynamics damping="0.7"/>
  </joint>

</robot>
```

### What's New?

*   **`<gazebo reference="base_link">`:** We've added a `<gazebo>` block for our `base_link`.
    *   **`<mu1>` and `<mu2>`:** These tags define the friction coefficients for the link's collision shape.
    *   **`<material>`:** We can also specify a Gazebo material to get more realistic rendering.
*   **`<dynamics damping="0.7"/>`:** Inside our `joint` definition, we've added a `<dynamics>` tag with a `damping` attribute. This simulates friction in the joint, causing it to slow down over time.

## Adding a Sensor

Now, let's give our robot a way to "see" the world. We will add a camera sensor to the arm.

```xml
<!-- ... (previous URDF content) ... -->

  <!-- Add a new link for the camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Attach the camera link to the arm link with a fixed joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="arm_link"/>
    <child link="camera_link"/>
    <origin xyz="0 0.05 0.25" rpy="0 0 0"/>
  </joint>

  <!-- Add a camera sensor to the camera_link -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>8.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/demo</namespace>
          <image_topic>image_raw</image_topic>
          <camera_info_topic>camera_info</camera_info_topic>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### What's New?

1.  **A new `<link>`:** We've created a small `camera_link` to represent the camera's physical body.
2.  **A new `<joint>`:** We've attached the `camera_link` to the `arm_link` with a `fixed` joint, so it moves with the arm.
3.  **A `<gazebo>` block for the camera:**
    *   **`<sensor name="camera" type="camera">`:** This defines a Gazebo sensor of type `camera`.
    *   **`<always_on>` and `<update_rate>`:** These tags control when and how often the sensor generates data.
    *   **`<camera>`:** This block contains the camera's intrinsic parameters, such as its field of view and image resolution.
    *   **`<plugin>`:** This is the most important part. We are using a pre-built Gazebo plugin, `libgazebo_ros_camera.so`, which comes with the `gazebo_ros_pkgs`. This plugin simulates the camera and publishes its images to ROS 2 topics. The `<ros>` block within the plugin tag configures the ROS 2 topics that the camera data will be published on. In this case, the image data will be on `/demo/image_raw`.

## What You've Learned

By using the `<gazebo>` tag, you can extend your URDF files to include a wealth of Gazebo-specific features. You've learned how to:
*   Add physics properties like friction and damping.
*   Add a sensor, like a camera, to your robot.
*   Use a pre-built Gazebo plugin to publish sensor data to ROS 2 topics.

## What's Next?

You have now created a simple robot with a sensor that can perceive its environment. The next logical step is to process that sensor data and make the robot act on it. This is the core of robotics: the "sense-plan-act" loop.

In the next module, you'll be introduced to **NVIDIA Isaac Sim**, a powerful, photorealistic robotics simulator that is tightly integrated with ROS 2 and provides advanced tools for perception and AI.
