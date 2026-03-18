import numpy as np
from pinocchio import SE3, neutral
from pyhpp.manipulation import (
    Device,
    Graph,
    Problem,
    urdf,
)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp_viser import Viewer


def display():
    v = Viewer(robot)
    v.initViewer(open=False, loadModel=True)
    v.setProblem(problem)
    v.setGraph(graph)
    return v


# use v = display() to create a Viewer instance.

robot = Device("tuto")

urdf_filename = "package://hpp_tutorial/urdf/staubli-drill.urdf"
srdf_filename = "package://hpp_tutorial/srdf/staubli-drill.srdf"

urdf.loadModel(
    robot, 0, "staubli", "anchor", urdf_filename, srdf_filename, SE3.Identity()
)

urdf_filename = "package://hpp_tutorial/urdf/square-plate.urdf"
srdf_filename = ""

urdf.loadModel(
    robot, 0, "plate", "freeflyer", urdf_filename, srdf_filename, SE3.Identity()
)
robot.setJointBounds(
    "plate/root_joint",
    [
        0,
        2,
        -1,
        1,
        0,
        2,
        -float("Inf"),
        float("Inf"),
        -float("Inf"),
        float("Inf"),
        -float("Inf"),
        float("Inf"),
        -float("Inf"),
        float("Inf"),
    ],
)

# Position the plate in the environment
q_init = neutral(robot.model())
r = robot.rankInConfiguration["plate/root_joint"]
q_init[r : r + 3] = [0.8, 0, 1]

# Add a handle on the plate
R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
T = np.array([0.02, 0, 0])
pose = SE3(rotation=R, translation=T)
robot.addHandle("plate/base_link", "plate/hole", pose, 0.1, 6 * [True])
handle = robot.handles()["plate/hole"]
handle.approachingDirection = np.array([0, 0, 1])

# Build the constraint graph
problem = Problem(robot)
graph = Graph("robot", robot, problem)
factory = ConstraintGraphFactory(graph)
factory.setGrippers(["staubli/tooltip"])
objects = ["plate"]
handlesPerObject = [["plate/hole"]]
contactsPerObject = [[]]
factory.setObjects(objects, handlesPerObject, contactsPerObject)
factory.generate()
graph.initialize()

shooter = problem.configurationShooter()
