# This message contains the state of the 
# Motion Planning module

Header header

uint32 state   # 0 -> none; 1 -> AT_THE_ORIGIN; 2 -> AT_THE_BIN; 
               # 3 -> MAPPED; 4 -> CLOSE_TO_OBJECT; 5 -> AWAY_FROM_OBJECT;
	       # 6 -> AT_THE_ORDER_BIN; 7 - GRASPED
