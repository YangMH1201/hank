import subprocess

def set_pose_gazebo_model(model_name, x, y, z, roll, pitch, yaw):
    try:
        # Command to set the pose of the model in Gazebo
        command = f"gz model -m {model_name} -x {x} -y {y} -z {z} -R {roll} -P {pitch} -Y {yaw}"

        # Execute the command using subprocess
        subprocess.run(command, shell=True, check=True)
        print(f"Successfully set the pose of {model_name} to (x, y, z) = ({x}, {y}, {z})")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        print(f"Failed to set the pose of {model_name}")

if __name__ == "__main__":
    model_name1 = "iris0"
    model_name2 = "iris1"
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    # set_pose_gazebo_model(model_name, x, y, z, roll, pitch, yaw)
    set_pose_gazebo_model("asphalt_plane", 0.0, 0.0,  0.0, 0.0, 0.0, 0.0)
    # model 1 tracker
    set_pose_gazebo_model(model_name1, 9.0, 9.0, z, roll, pitch, yaw)
    # model 2 target
    set_pose_gazebo_model(model_name2, 0.0, 0.0, z, roll, pitch, yaw)