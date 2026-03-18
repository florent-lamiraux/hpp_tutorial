# Generate configuration in pregrasp reachable from q_init and in grasp reachable from pregrasp
transition = graph.getTransition("staubli/tooltip > plate/hole | f_01")
pv = transition.pathValidation()
res, qpg, err = graph.generateTargetConfig(transition, q_init, q_init)  # -> failure
if not res:
    print("Failed to project q_init to pregrasp waypoint state")

for i in range(1000):
    transition = graph.getTransition("staubli/tooltip > plate/hole | f_01")
    q = shooter.shoot()
    res, qpg, err = graph.generateTargetConfig(transition, q_init, q)
    if not res:
        continue
    res, report = pv.validateConfiguration(qpg)
    if not res:
        continue
    transition = graph.getTransition("staubli/tooltip > plate/hole | f_12")
    res, qg, err = graph.generateTargetConfig(transition, qpg, qpg)
    if res:
        break

# Notice that qg is in collision
res, report = pv.validateConfiguration(qg)
print(report)
