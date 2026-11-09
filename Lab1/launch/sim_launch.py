from simple_launch import SimpleLauncher
import os

sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('gui', True)
full_world = os.path.abspath(os.path.dirname(__file__) + '/../urdf/world_full.sdf')


def launch_setup():

    gz_args = '-r' if sl.arg('gui') else '-r -s'

    if os.path.exists(full_world):
        sl.gz_launch(full_world, gz_args)
    else:
        sl.gz_launch(sl.find('ecn_auv_lab', 'world.sdf'), gz_args)
        sl.save_gz_world(full_world, 5.)

    # spawn the turbine
    sl.include('floatgen', 'farm_launch.py', launch_arguments={'gz': False, 'x': -200, 'y': -20, 'yaw': 0.})

    # spawn terrain
    with sl.group(ns = 'terrain'):
        sl.robot_state_publisher('ecn_auv_lab', 'terrain.xacro')
        sl.spawn_gz_model('terrain')

    # spawn BlueROV2
    sl.include('ecn_auv_lab', 'bluerov2_launch.py')

    # display in RViz
    sl.rviz(sl.find('ecn_auv_lab', 'layout.rviz'))

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
