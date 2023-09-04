Control System Example with learned hysteresis

This example uses a simple control system that builds a local linear model of behavior from impulse-response changes over time.  The class allows for eather a target value to be maintained, or a optimal (min or max) value to be seeked.  The test illustrates both styles.   In the test example, a toy system is set up where there are some variabilites such as a long running sin function to vary the measurements, or an ever-decreasing overall signal for the signal maximization problem.  In each case, the system quickly learns the change needed to return the system to optimal.  Included is a buffer threshold for which the control system makes no corrections, in order to minimize changes to the system when at or near optimal.

