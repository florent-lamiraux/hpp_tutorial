# Defining security margins
from pyhpp.manipulation.security_margins import SecurityMargins

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
pv = transition.pathValidation()
res, report = pv.validateConfiguration(qg)
print(report)

# Generate configuration in pregrasp reachable from q_init and in grasp reachable from pregrasp
# and test collision for both.
for i in range(1000):
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
    # Project pregrasp configuration on grasp state
    transition = graph.getTransition("staubli/tooltip > plate/hole | f_12")
    res, qg, err = graph.generateTargetConfig(transition, qpg, qpg)
    if not res:
        continue
    # Check the configuration for collision
    pv = transition.pathValidation()
    res, report = pv.validateConfiguration(qg)
    if res:
        break

# Plan motions between waypoint configurations
planner = TransitionPlanner(problem)
planner.maxIterations(1000)
for i in range(10):
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
    # plan motion between q_init and qpg
    planner.setTransition(transition)
    try:
        q_goal = np.zeros((1, robot.configSize()), order="F")
        q_goal[0, :] = qpg
        p1 = planner.planPath(q_init, q_goal, True)
    except Exception as exc:
        print(f"path planning failed between q_init and qpg: {exc}")
        continue
    # Project pregrasp configuration on grasp state
    transition = graph.getTransition("staubli/tooltip > plate/hole | f_12")
    res, qg, err = graph.generateTargetConfig(transition, qpg, qpg)
    if not res:
        continue
    # Check the configuration for collision
    pv = transition.pathValidation()
    res, report = pv.validateConfiguration(qg)
    if not res:
        continue
    # plan motion between qpg and qg
    planner.setTransition(transition)
    try:
        q_goal = np.zeros((1, robot.configSize()), order="F")
        q_goal[0, :] = qg
        p2 = planner.planPath(qpg, q_goal, True)
    except Exception as exc:
        print(f"path planning failed between qpg and qg: {exc}")
        continue
    if p2:
        p3 = p2.reverse()
        break
