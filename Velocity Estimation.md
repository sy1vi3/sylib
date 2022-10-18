# Smart Motor Velocity Estimation

Sylib has its own system for calculating a V5 Smart Motor's velocity from information provided by its internal encoder. 

In PROS, the get_actual_velocity function is a bit of a misnomer.
It doesn't get the "actual" motor velocity, it gets the value reported by the motor itself.
This value is calculated by a currently unknown process running onboard the motor.
The motor-reported velocity value is incredibly noisy, and as such is not really appropriate
for precise velocity control like is needed on a flywheel.

The motors also have a method for finding the raw tick count of their internal encoders.
This value is much less noisy, but it is also problematic in its own way.
Internally, the motors seem to update their telemetry information every 5ms,
but this information is only made available to the user program every 10ms,
meaning that at the fastest rate that the user program can poll the motors,
the new information provided makes up 2 update frames worth of data.

Or at least, this is how it's supposed to work.
In reality, the motors don't update every 5ms. They update almost every 5ms.
About 1 in every 20 times, the time gap between frames is 4ms, and rarer still it takes 6ms.

Most of the time this is not a problem, because if the motor updates a few
milliseconds before sending information to the brain, the ticks that happen
in that time are accounted for the next time.

Because sometimes update frames take 4ms while the user code is assuming they
always take 5, a drift occurs. The motor's internal clock drifts from the clock
on the brain by about 1 part in 95, at least on the motors I tested.

Averaged over a long period of time, this doesn't matter.
But when attempting to find instantaneous (ish) motor velocity information,
it matters a lot.

Every so often, this drift ticks over the boundary of when information is sent to the brain,
and the brain's polling and the motor's internal updating are back in sync.
When this happens, one 10ms time period from the brain's perspective contains only 5ms
worth of data, followed shortly (one or two updated later) by a 10ms time
period consisting of 15ms worth of data.

When calculating velocity by taking the difference in motor ticks every 10ms and dividing
by time, this results in every so often the calculated velocity value being 0.5x what
it actually is, mirrored shortly after by the calculation outputting a value 1.5x where
it should be.

In order to address this, in addition to polling for tick count information,
sylib also polls motors for their own internal record of what time their information
was most recently updated.

Instead of using the time difference between updates of the loop in the user program,
sylib considers dT equal to the difference in time between motor timestamps every loop.
Most of the time these align, but every so often when the drift ticks over
the critical boundary, the program is able to detect this and use the actual
value for calculating velocity.

Since this measurement is based on an encoder with 25 ticks per rotation,
a difference of one tick more or less results in a noticable jump in calculated velocity.
Since the motors will obviously not spin at exactly a whole number of ticks per update,
the value will regularly vary ±1 tick.

Sylib uses several kinds of filters to take the raw velocity calculation and turn it
into something usable. It primarily used a 3-tap simple moving average, meaning that
it considers the current and last values, and outputs their mean.

this gets most of the way there, but the value still isn't smooth like it should be.
To accomplish smoothing, the output of the sma filter is fed into an exponential moving
average filter, which outputs the current input value times a constant, summed with
the previous output value multipled by 1 minus that constant.A lower constant will result
in a smoother line but higher latency, and vice-versa.

When the motor velocity is relatively stable, using a low constant is beneficial,
but when the motor is changing speeds quickly a higher constant is needed.

Sylib makes a rough estimation of motor acceleration, and uses this to increase
or decrease the ema constant. When acceleration spikes, and for about 200ms after,
sylib uses a higher multiplier, and when acceleration is low it uses a lower one.
This works to very quickly adjust to an actual change in motor velocity, but
still be resilient to small deviations when otherwise mostly stable. 

Since acceleration is a derivation of an already-derived value, the data isn't
particularly good, but it's good enough for what is needed.

Just for fun, sylib also provides a method to get motor jerk, which is probably
a completely garbage number, as well as snap (which I know for a fact is useless).