# Velocity Control

Precise velocity controllers are vital to the consistency of many different kinds of robot subsystems.
The V5 motors have an internal PID controller used to set motor velocity
(as well as position), but at least for velocity this control is completely awful.

Sylib does not provide a velocity controller for teams to use.
Understanding and tuning control loops are a vital thing to learn in robotics,
and providing these for free is detrimental to everyone involved.

Instead, sylib provides an optional template for users to make their own velocity controllers. You can provide your own constants for P, I, D, Feedforward, and TBH controllers, and sylib will implement them automatically.
These controllers automatically use sylib's own motor velocity estimators, rather than the imprecise one provided by the motors.

