# zoompilot

A Mazda-optimized fork of [sunnypilot](https://github.com/sunnypilot/sunnypilot) for the CX-5 and CX-9. My goal is to enable the best steering possible and support all sunnypilot features on the 2022-2025 CX-5, staying within openpilot's safety guidelines.

This is what I run on my own 2022 CX-5 every day. There's a nicer tour of all of this at [zoompilot.ai](https://zoompilot.ai).

## Install

When your comma device asks for a custom software URL during setup, type:

```
zoompilot/main
```

`main` carries the releases, built ahead of time so the device doesn't have to compile on install. `develop` is where the day to day work lands if you want it earlier. Devices installed from my old personal fork move themselves to `main` on their next start.

**This is experimental software.** You drive the car, you follow the law where you live, and you carry the risk. It ships with no warranty and no liability for any damage or injury.

## steering improvements

Using a data driven approach I reverse engineered the steering hardware and developed features to use the EPS to its full potential.

- **Speed-dependent torque.** The EPS caps the steering torque it will deliver, and that cap changes with speed. We encode the whole curve, so openpilot knows the extra torque it has at low speeds and the reduced torque at high speeds. More confident in the neighborhood, fewer wobbles on the highway.
- **Speed-dependent tuning.** Stock openpilot learns ONE lateral acceleration factor and ONE friction value for every speed. We let it learn across seven speed ranges instead, matching the EPS's non-linear speed-dependent behaviour.
- **Rate-matched commands.** The EPS accepts steering commands changing by up to 12 units a frame; stock asked for 10, leaving ~17% of that rate unused. We ask for the full 12, so torque ramps in faster.
- **Factory-matched specs.** Steering ratio, mass, wheelbase, and steering lag, set to Mazda's real figures and refined against thousands of miles of learned data, so commands land where the planner intends.

## fix annoyances

The stock port threw warnings that should have only applied to pre-2022 cars. I pulled the logs from many drivers and cut the false ones. The real ones still fire.

- **Place hands on wheel alert.** The Mazda port was built on pre-2022 cars, where an LKAS_BLOCK signal meant steering control was lost. Across thousands of logged miles from many drivers, we found 2022+ EPS keeps steering through LKAS_BLOCK. It mostly fires near a stop, or when stock LKAS loses the lane lines. Real faults still disengage; we just stopped alerting when the car is still steering.
- **Steering override hysteresis.** The same override filter Hyundai, Ford, Tesla, and Rivian run. It weighs more frames before deciding you've taken the wheel, so a pothole or rough patch no longer trips a phantom takeover.

## wire up sensors

Your CX-5 ships with sensors the stock port never reads. We wire them in.

- **Forward radar.** A radar behind the front badge runs the factory cruise. We read it too: distance, angle, and closing speed for up to four cars ahead, fused with the camera. Braking and throttle stay factory.
- **Blind-spot monitors.** The factory blind-spot sensors already watch the lanes beside you. We feed what they see into openpilot before every lane change.
- **Speed-limit signs.** The camera already reads posted limits for your dash. We route those into speed-limit assist, which can set your cruise to match, so you're not re-dialing at every sign.

## radar cruise enhancements

The CX-5 keeps its factory cruise; zoompilot can set the speed for you by 'pressing' your wheel buttons.

- **Fixed ICBM.** sunnypilot's Intelligent-Cruise-Button-Management and Smart-Cruise are broken for Mazda. I rebuilt how zoompilot works your wheel buttons. The speed you dial is remembered exactly: curves and speed limits can borrow it for a while, but you get your number back, never one or two under it. If the car misses a press, it quietly catches up.
- **Speed-limit assist that sticks.** When the car sees a new limit, the screen asks once. Tap minus to accept a lower one and zoompilot dials the car down for you. It used to forget your answer a moment later; now it holds until the road changes. Press plus while it has you at a limit and it steps aside until the next sign. Your buttons always win.
- **Smart Cruise.** A sunnypilot feature that reduces your set speed before a curve in the road and sets it back after. You can use vision or downloaded maps to determine when to slow down. Enable it in the cruise settings menu.
- **Deceleration overshoot (alpha).** The Mazda does not instantly react to adjustments in set cruise speed. This option reduces the set cruise speed MORE than what the model calls for, to get the deceleration the curve needs.

## additional Mazdas covered

Not just the CX-5.

- **EPS swaps.** Swapped a 2022+ CX-5 steering rack into an older Mazda? We fingerprint the rack by its firmware, not by the car openpilot thinks you're driving, and steer it all the way to a stop.
- **CX-9.** Supported for all features, but speed-dependent torque will take longer to learn since I only supply seeds for the CX-5. The CX-9 specs are also updated to match Mazda's official figures.

## recommended setup

1. Factory-reset the device before you install. A clean device carries no stale settings from a previous fork.
2. Pick a driving model. I run Firehose. DTRv6 is a favourite and MacroStiff is great at high speed.
3. Under steering: turn on torque control, then self-tune, then speed-dependent self-tune.
4. Leave custom tune and manual real-time off. Let the car teach the software. That's the point.

Want to slow down for curves? Under cruise, turn on intelligent cruise button management, then pick smart cruise vision, maps, or both. Maps need a region downloaded through SunnyLink first.

You can manage almost all of it from the device screen. No laptop, no cloud editor. SunnyLink still works if you like it.

## issues

Something misbehaving on your Mazda? Open an issue on the [tracker](https://github.com/zoompilot/zoompilot/issues). A route ID or dashcam clip helps a lot.

## credits

zoompilot stands on [sunnypilot](https://github.com/sunnypilot/sunnypilot), which stands on [openpilot](https://github.com/commaai/openpilot) by comma.ai. Most of the code here is theirs. Remote access and dashboards come from [sunnylink](https://www.sunnylink.ai/), a free service paid for by the sunnypilot project. To support them: [sponsor sunnypilot](https://github.com/sponsors/sunnyhaibin), or [buy hardware from comma](https://comma.ai/shop).

## license

This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed under a custom license requiring permission for use. See [LICENSE.md](LICENSE.md) for sunnypilot's terms, [LICENSE](LICENSE) for openpilot's, and [NOTICE.md](NOTICE.md) for how they stack up and what zoompilot's own files are under.

---

These features would work on other vehicles and could be upstreamed into sunnypilot or openpilot. I'm slowly working on that, but it's easier to share my own fork in the meantime.

Mazda, comma.ai, and the sunnypilot project neither endorse this nor have anything to do with it.

zoom-zoom-zoom
