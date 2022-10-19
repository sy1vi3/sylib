# Background Services

Sylib contains its own daemon which runs in the background, in a single Task
as defined by either PROS or VEXcode, depending on where it is running. 
This daemon takes care of actually sending commands to motors and other devices periodically.

The daemon contains a loop which cycles every 2ms, specially timed to not interfere with VEX's
own background processing functions. This also allows the motor velocity estimator and speed controllers
to update without the user having to worry about their own multitasking. 

Public methods of the `Motor` class do not directly interact with the VEX SDK. Instead, they
update internally stored values which are then communicated with the SDK by the motor's update function
every 5 ticks of the sylib daemon's loop, meaning every 10ms. It is not possible to send or receive information
from V5 devices faster than this anyways.

The same applies for the `Addrled` class, except they are set to only run their update function every 6*n frames,
where n is the total number of addrled devices configured. The reason for this is that the VEX hardware will not
allow user code to update information for lights plugged into an ADI expander on the same smart port faster than
every 10ms. It will ignore anything sent faster than this. 

In order to address (haha get it) this, the public methods of the `Addrled` class do not directly
tell the hardware what to do with the lights. Instead, it updates internally stored values of the object,
which are then communicated to the lights themselves by the update function. This allows users to update the colors
of light strips rapidly without needing to worry about commands being skipped, because sylib will only actually
talk to a single addrled device every 12ms. The extra 2 milliseconds is to be safe, because sometimes clock inconsistencies
can result in a frame taking 1ms or 3ms instead of 2ms. 

`Motor`s and `Addrled`s both derive from the `Device` class. It is possible for users to make their own
classes that inherit from the `Device` class in order to make use of sylib's background services for novel
purposes. It is important to keep in mind if there is blocking code in the `update` function, it will cause issues
for all sylib services, as the daemon runs in a single thread. Anyone going to the effort of doing this likely knows
how to manage this themselves.

