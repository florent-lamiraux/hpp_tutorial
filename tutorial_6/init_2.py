import numpy as np
from pinocchio import SE3, neutral
from pyhpp.constraints import ComparisonType, ComparisonTypes
from pyhpp.core import SimpleTimeParameterization, SplineGradientBased_bezier3
from pyhpp.manipulation import (
    Device,
    Graph,
    Problem,
    TransitionPlanner,
    urdf,
)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins
from pyhpp.manipulation.steering_method import Cartesian, makePiecewiseLinearTrajectory
from pyhpp_toppra import Toppra
from pyhpp_rviz import RVizVisualizer

from pyhpp.manipulation import RandomShortcut
from pyhpp.core import PathOptimizer
from pyhpp.core.path import Vector as PathVector
from pyhpp.core.path import Path


def display():
    v = RVizVisualizer()
    v.initViewer(robot)
    v.setProblem(problem)
    v.setGraph(graph)
    return v


# use v = display() to create an RVizVisualizer instance.

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

# Load an obstacle between robot and plate to force a non-straight path
obstacle_pose = SE3(rotation=np.identity(3), translation=np.array([0.5, -0.2, 1.2]))
urdf.loadModel(
    robot,
    0,
    "obstacle",
    "anchor",
    "package://hpp_tutorial/urdf/obstacle.urdf",
    "",
    obstacle_pose,
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
# Set security margins
sm = SecurityMargins(problem, factory, ["staubli", "plate"], robot)
sm.setSecurityMarginBetween("staubli", "plate", 0.05)
sm.apply()
# Deactivate collision checking between robot last joint and plate for the last part of the
# motion
transition = graph.getTransition("staubli/tooltip > plate/hole | f_12")
graph.setSecurityMarginForTransition(
    transition, "staubli/joint_6", "plate/root_joint", float("-inf")
)
graph.initialize()
problem.constraintGraph(graph)

shooter = problem.configurationShooter()

# Plan motions between waypoint configurations
planner = TransitionPlanner(problem)
planner.maxIterations(1000)
p1 = p2 = p3 = p4 = None

for i in range(50):
    # Project random configuration on pregrasp waypoint state
    transition = graph.getTransition("staubli/tooltip > plate/hole | f_01")
    q = shooter.shoot()
    res, qpg, err = graph.generateTargetConfig(transition, q_init, q)
    if not res:
        continue
    # Check the configuration for collision
    pv = transition.pathValidation()
    res, report = pv.validateConfiguration(qpg)
    if not res:
        continue
    # Plan motion between q_init and qpg
    planner.setTransition(transition)
    try:
        q_goal = np.zeros((1, robot.configSize()), order="F")
        q_goal[0, :] = qpg
        p1 = planner.planPath(q_init, q_goal, True)
    except Exception as exc:
        print(f"path planning failed between q_init and qpg: {exc}")
        continue

    # Build cartesian drilling path between qpg and qg (tutorial 4)
    gripper = robot.grippers()["staubli/tooltip"]
    trajConstraint = handle.createGrasp(gripper, "drilling")
    cts = ComparisonTypes()
    cts[:] = 6 * [ComparisonType.Equality]
    trajConstraint.comparisonType(cts)
    init = trajConstraint.function()(qpg)
    goal = np.array([0, 0, 0, 0, 0, 0, 1])

    weights = 50 * np.ones(6)
    points = np.zeros(14).reshape((2, 7))
    points[0:] = init.vector()
    points[1:] = goal
    rhs = makePiecewiseLinearTrajectory(points, weights)

    cartesian = Cartesian(planner.innerProblem())
    cartesian.trajectoryConstraint = trajConstraint
    cartesian.setRightHandSide(rhs, True)
    res, p2_raw = cartesian.planPath(qpg)
    if not res:
        continue

    # Validate p2_raw for collision (tutorial 4)
    transition_12 = graph.getTransition("staubli/tooltip > plate/hole | f_12")
    pv2 = transition_12.pathValidation()
    res, p2_valid, report = pv2.validate(p2_raw, False)
    if not res:
        continue

    # Smooth p1 with cubic Bézier spline (tutorial 5)
    bezier = SplineGradientBased_bezier3(problem)
    p1_smooth = bezier.optimize(p1)

    # Time-parameterize both paths with TOPPRA (tutorial 5)
    toppra = Toppra(problem)
    toppra.velocityScale = 0.5
    toppra.effortScale = -1
    toppra.N = 100
    toppra.accelerationLimits = np.array(12 * [0.5])
    p1 = toppra.optimize(p1_smooth)
    p2 = p2_valid

    print(f"Path p1 (q_init -> qpg), TOPPRA duration: {p1.length():.3f} s")
    print(f"Path p2 (qpg   -> qg ), TOPPRA duration: {p2.length():.3f} s")
    break


from pyhpp.core.path import Vector as PathVector

# Créer un PathVector vide
full_path = PathVector(robot.configSize(), robot.numberDof())

# Ajouter les chemins dans l'ordre
full_path.appendPath(p1)
full_path.appendPath(p2)
full_path.appendPath(p2.reverse())
full_path.appendPath(p1.reverse())
