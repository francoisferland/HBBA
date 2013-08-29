# An IW event.

uint8   DES_ON      = 1 # The desire appeared in the set.
uint8   DES_OFF     = 2 # The desire disappeared from the set.
uint8   INT_ON      = 3 # The desire appeared in the intention.
uint8   INT_OFF     = 4 # The desire disappeared from the intention.
uint8   EXP_ON      = 5 # The desire's exploitation has started.
uint8   EXP_OFF     = 6 # The desire's exploitation stopped.
uint8   EXP_TIMEOUT = 7 # The desire has not been exploited before timeout.
uint8   ACC_ON      = 8 # The desire's goal is or has been accomplished.
uint8   ACC_OFF     = 9 # The desire's goal has stopped being accomplished.

string  desire      # The desire associated with the event.
string  desire_type # The desire's class.
uint8   type        # The event type, see constants for possibilities.
