"""
Generate a minimal 3D biped URDF with hip roll + hip pitch + knee + ankle per leg.
"""
import os

URDF_TEMPLATE = """<?xml version="1.0"?>
<robot name="biped">
  <!-- Pelvis (base link) -->
  <link name="pelvis">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{pelvis_mass}"/>
      <inertia ixx="{Ipxx}" ixy="0" ixz="0" iyy="{Ipyy}" iyz="0" izz="{Ipzz}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{pelvis_w} {pelvis_d} {pelvis_h}"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.4 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{pelvis_w} {pelvis_d} {pelvis_h}"/>
      </geometry>
    </collision>
  </link>

  <!-- Left upper leg (hip roll to hip pitch dummy) -->
  <link name="left_upper_leg">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="left_hip_roll" type="revolute">
    <parent link="pelvis"/>
    <child link="left_upper_leg"/>
    <origin xyz="0 {hip_y} {neg_hip_z_offset}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="{neg_hip_roll_limit}" upper="{hip_roll_limit}" effort="100" velocity="10"/>
  </joint>

  <!-- Left Thigh -->
  <link name="left_thigh">
    <inertial>
      <origin xyz="0 0 {neg_thigh_len_half}" rpy="0 0 0"/>
      <mass value="{thigh_mass}"/>
      <inertia ixx="{Itxx}" ixy="0" ixz="0" iyy="{Ityy}" iyz="0" izz="{Itzz}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 {neg_thigh_len_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{thigh_w} {thigh_d} {thigh_len}"/>
      </geometry>
      <material name="grey">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 {neg_thigh_len_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{thigh_w} {thigh_d} {thigh_len}"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_pitch" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="{neg_hip_pitch_limit}" upper="{hip_pitch_limit}" effort="100" velocity="10"/>
  </joint>

  <!-- Left Shin -->
  <link name="left_shin">
    <inertial>
      <origin xyz="0 0 {neg_shin_len_half}" rpy="0 0 0"/>
      <mass value="{shin_mass}"/>
      <inertia ixx="{Isxx}" ixy="0" ixz="0" iyy="{Isyy}" iyz="0" izz="{Iszz}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 {neg_shin_len_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{shin_w} {shin_d} {shin_len}"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 {neg_shin_len_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{shin_w} {shin_d} {shin_len}"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 {neg_thigh_len}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="{knee_min}" upper="{knee_max}" effort="100" velocity="10"/>
  </joint>

  <!-- Left Foot -->
  <link name="left_foot">
    <inertial>
      <origin xyz="{foot_cog_x} 0 {neg_foot_h_half}" rpy="0 0 0"/>
      <mass value="{foot_mass}"/>
      <inertia ixx="{Ifxx}" ixy="0" ixz="0" iyy="{Ifyy}" iyz="0" izz="{Ifzz}"/>
    </inertial>
    <visual>
      <origin xyz="{foot_cog_x} 0 {neg_foot_h_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{foot_l} {foot_w} {foot_h}"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="{foot_cog_x} 0 {neg_foot_h_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{foot_l} {foot_w} {foot_h}"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_ankle" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 {neg_shin_len}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="{neg_ankle_limit}" upper="{ankle_limit}" effort="100" velocity="10"/>
  </joint>

  <!-- Right upper leg -->
  <link name="right_upper_leg">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="right_hip_roll" type="revolute">
    <parent link="pelvis"/>
    <child link="right_upper_leg"/>
    <origin xyz="0 {neg_hip_y} {neg_hip_z_offset}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="{neg_hip_roll_limit}" upper="{hip_roll_limit}" effort="100" velocity="10"/>
  </joint>

  <!-- Right Thigh -->
  <link name="right_thigh">
    <inertial>
      <origin xyz="0 0 {neg_thigh_len_half}" rpy="0 0 0"/>
      <mass value="{thigh_mass}"/>
      <inertia ixx="{Itxx}" ixy="0" ixz="0" iyy="{Ityy}" iyz="0" izz="{Itzz}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 {neg_thigh_len_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{thigh_w} {thigh_d} {thigh_len}"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 {neg_thigh_len_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{thigh_w} {thigh_d} {thigh_len}"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_pitch" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_thigh"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="{neg_hip_pitch_limit}" upper="{hip_pitch_limit}" effort="100" velocity="10"/>
  </joint>

  <!-- Right Shin -->
  <link name="right_shin">
    <inertial>
      <origin xyz="0 0 {neg_shin_len_half}" rpy="0 0 0"/>
      <mass value="{shin_mass}"/>
      <inertia ixx="{Isxx}" ixy="0" ixz="0" iyy="{Isyy}" iyz="0" izz="{Iszz}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 {neg_shin_len_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{shin_w} {shin_d} {shin_len}"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 {neg_shin_len_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{shin_w} {shin_d} {shin_len}"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 {neg_thigh_len}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="{knee_min}" upper="{knee_max}" effort="100" velocity="10"/>
  </joint>

  <!-- Right Foot -->
  <link name="right_foot">
    <inertial>
      <origin xyz="{foot_cog_x} 0 {neg_foot_h_half}" rpy="0 0 0"/>
      <mass value="{foot_mass}"/>
      <inertia ixx="{Ifxx}" ixy="0" ixz="0" iyy="{Ifyy}" iyz="0" izz="{Ifzz}"/>
    </inertial>
    <visual>
      <origin xyz="{foot_cog_x} 0 {neg_foot_h_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{foot_l} {foot_w} {foot_h}"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="{foot_cog_x} 0 {neg_foot_h_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{foot_l} {foot_w} {foot_h}"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_ankle" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 {neg_shin_len}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="{neg_ankle_limit}" upper="{ankle_limit}" effort="100" velocity="10"/>
  </joint>
</robot>
"""


def compute_inertia_box(m, w, d, h):
    """Box inertia about its center."""
    ixx = m * (d**2 + h**2) / 12.0
    iyy = m * (w**2 + h**2) / 12.0
    izz = m * (w**2 + d**2) / 12.0
    return ixx, iyy, izz


def create_biped_urdf(path: str = "biped.urdf"):
    """
    Write a simplified biped URDF to `path`.
    Legs: hip_roll, hip_pitch, knee_pitch, ankle_pitch.
    Returns the absolute path.
    """
    # Dimensions tuned for com(z) ~ 0.85 and step parameters from pattern_generator
    pelvis_w, pelvis_d, pelvis_h = 0.15, 0.20, 0.10
    pelvis_mass = 20.0

    thigh_len, thigh_w, thigh_d = 0.37, 0.06, 0.06
    thigh_mass = 5.0

    shin_len, shin_w, shin_d = 0.37, 0.05, 0.05
    shin_mass = 3.0

    foot_l, foot_w, foot_h = 0.20, 0.10, 0.04
    foot_mass = 1.0
    foot_cog_x = 0.03

    hip_y = 0.10
    hip_z_offset = pelvis_h / 2.0

    hip_roll_limit = 0.8
    hip_pitch_limit = 1.2
    knee_min = -2.2
    knee_max = 0.1
    ankle_limit = 0.8

    Ipxx, Ipyy, Ipzz = compute_inertia_box(pelvis_mass, pelvis_w, pelvis_d, pelvis_h)
    Itxx, Ityy, Itzz = compute_inertia_box(thigh_mass, thigh_w, thigh_d, thigh_len)
    Isxx, Isyy, Iszz = compute_inertia_box(shin_mass, shin_w, shin_d, shin_len)
    Ifxx, Ifyy, Ifzz = compute_inertia_box(foot_mass, foot_l, foot_w, foot_h)

    neg_hip_y = -hip_y
    neg_hip_z_offset = -hip_z_offset
    neg_thigh_len_half = -thigh_len / 2.0
    neg_shin_len_half = -shin_len / 2.0
    neg_foot_h_half = -foot_h / 2.0
    neg_thigh_len = -thigh_len
    neg_shin_len = -shin_len
    neg_hip_roll_limit = -hip_roll_limit
    neg_hip_pitch_limit = -hip_pitch_limit
    neg_ankle_limit = -ankle_limit

    urdf = URDF_TEMPLATE.format(
        pelvis_w=pelvis_w, pelvis_d=pelvis_d, pelvis_h=pelvis_h,
        pelvis_mass=pelvis_mass, Ipxx=Ipxx, Ipyy=Ipyy, Ipzz=Ipzz,
        thigh_len=thigh_len, thigh_w=thigh_w, thigh_d=thigh_d,
        thigh_mass=thigh_mass, Itxx=Itxx, Ityy=Ityy, Itzz=Itzz,
        shin_len=shin_len, shin_w=shin_w, shin_d=shin_d,
        shin_mass=shin_mass, Isxx=Isxx, Isyy=Isyy, Iszz=Iszz,
        foot_l=foot_l, foot_w=foot_w, foot_h=foot_h,
        foot_mass=foot_mass, foot_cog_x=foot_cog_x,
        Ifxx=Ifxx, Ifyy=Ifyy, Ifzz=Ifzz,
        hip_y=hip_y, hip_z_offset=hip_z_offset,
        hip_roll_limit=hip_roll_limit, hip_pitch_limit=hip_pitch_limit,
        knee_min=knee_min, knee_max=knee_max, ankle_limit=ankle_limit,
        neg_hip_y=neg_hip_y, neg_hip_z_offset=neg_hip_z_offset, neg_thigh_len_half=neg_thigh_len_half,
        neg_shin_len_half=neg_shin_len_half, neg_foot_h_half=neg_foot_h_half,
        neg_thigh_len=neg_thigh_len, neg_shin_len=neg_shin_len,
        neg_hip_roll_limit=neg_hip_roll_limit, neg_hip_pitch_limit=neg_hip_pitch_limit,
        neg_ankle_limit=neg_ankle_limit,
    )

    with open(path, "w") as f:
        f.write(urdf)
    return os.path.abspath(path)


if __name__ == "__main__":
    p = create_biped_urdf("biped.urdf")
    print(f"URDF written to: {p}")
