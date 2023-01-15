import pybullet as p
import pybullet_data
import os
import time


NUM_SIMULATION_STEPS = 20000

def get_case_gravity_orientation(mesh_path, show_gui=False):
    # Connect to the physics engine
    if show_gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    p.setGravity(0, 0, -10)

    # Load a plane as the ground
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF('plane.urdf')

    # Create URDF
    create_urdf_from_template(mesh_path)
    mesh_id = p.loadURDF(f'{mesh_path}.urdf', [0, 0, 50])
    os.remove(f'{mesh_path}.urdf')

    # Run the simulation for NUM_SIMULATION_STEPS steps
    print(f'Running simulation for {NUM_SIMULATION_STEPS} iterations, or 5 seconds..')

    t = time.time()
    i = 0
    while time.time() - t < 10 and i < NUM_SIMULATION_STEPS:
        p.stepSimulation()
        i += 1
        if show_gui:
            time.sleep(0.01)
    print("Done simulating")

    _, orientation = p.getBasePositionAndOrientation(mesh_id)

    p.disconnect()
    return orientation


def create_urdf_from_template(mesh_name):
    with open('src/template.urdf', 'r') as template:
        template_text = template.read()

    mesh_urdf = template_text.replace('!TEMPLATE!', mesh_name)

    with open(f"{mesh_name}.urdf", "w") as f:
        f.write(mesh_urdf)
