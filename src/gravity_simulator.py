import pybullet as p
import pybullet_data
import time
import pymesh


def get_case_gravity_orientation(mesh_name):
    # Create URDF
    write_urdf_text(mesh_name)
    # Connect to the physics engine
    physicsClient = p.connect(p.GUI)

    # Set the gravity in the simulation
    p.setGravity(0, 0, -10)

    # Load a plane as the ground
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    # Load a box as the object
    boxId = p.loadURDF(f"{mesh_name}.urdf", [0, 0, 2])

    # Run the simulation for 100 steps
    for i in range(10000):
        p.stepSimulation()


    position, orientation = p.getBasePositionAndOrientation(boxId)

    # Disconnect from the physics engine
    p.disconnect()
    # orientation = p.getEulerFromQuaternion(orientation)
    return orientation


def write_urdf_text(mesh_name):
    print("Creation of <{}>...".format(mesh_name), end="")
    with open('src/template.urdf', 'r') as template:
        template_text = template.read()

    mesh_urdf = template_text.replace('!TEMPLATE!', mesh_name)

    with open(f"{mesh_name}.urdf", "w") as f:
        f.write(mesh_urdf)
        print("done")
