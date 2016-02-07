

================================================================================================================
HIGH-LEVEL DECISION MAKING
================================================================================================================

- State Machine testing
  Run roscore in a terminal
  $ roscore
  
  Run the decision making module in another
  $ source /home/alberto/RUTGERS/apc_hg/devel/setup.sh {sp alias for short}
  $ rosrun prx_decision_making prx_decision_making_node
  
  Then run several programs to test it
  $ source /home/alberto/RUTGERS/apc_hg/devel/setup.sh {sp alias for short}
  $ rostopic list
  $ rosmsg show prx_decision_making/DecisionMakingStateMessage
  $ rostopic hz /decision_making_state
  $ rostopic echo /decision_making_state


