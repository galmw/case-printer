import pybullet as p
import pybullet_data
import os


def get_case_gravity_orientation(mesh_path):
    # Connect to the physics engine
    p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    # Load a plane as the ground
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF('plane.urdf')

    # Create URDF
    create_urdf_from_template(mesh_path)
    mesh_id = p.loadURDF(f'{mesh_path}.urdf', [0, 0, 2])
    os.remove(f'{mesh_path}.urdf')

    # Run the simulation for 10000 steps
    for i in range(10000):
        p.stepSimulation()

    position, orientation = p.getBasePositionAndOrientation(mesh_id)

    p.disconnect()
    return orientation


def create_urdf_from_template(mesh_name):
    with open('src/template.urdf', 'r') as template:
        template_text = template.read()

    mesh_urdf = template_text.replace('!TEMPLATE!', mesh_name)

    with open(f"{mesh_name}.urdf", "w") as f:
        f.write(mesh_urdf)
