# chargepal_behaviour_tree
This package is the executor of chargepal jobs.

## Dependencies

- chargepal_actions : Perfroming any ros actions affiliated to the robot
- chargepal_services : Perfroming any ros services affiliated to the robot
- chargepal_client : Communcation medium to the grpc server

The figure below shows the dependecies as a flow state:
[Dependencies](/images/dependecies.png)

## Overview
The executor asks the server for a job every 1 second. The job description, the sequence of actions for a specific job type and the sequence of actions for a failed action is shown in this [document](/Chargepal Actions.pdf).

The figure below shows the flow inside the behaviour tree.
[Executor](/images/executor.png)
