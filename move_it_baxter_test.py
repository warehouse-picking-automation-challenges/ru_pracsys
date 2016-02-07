from moveit_commander import MoveGroupCommander

if __name__ == '__main__':
  group = MoveGroupCommander("left_arm")
  
  # move to a random target
  group.set_random_target()
  group.go()
