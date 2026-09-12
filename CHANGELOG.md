zoompilot v2026.09.12-14
========================
* adjustments to address LKAS errors.
* On cars with TJA/CTS, zoompilot now switches off TJA fully so it doesn't take over when you disengage zoompilot. This also addresses related "Front Camera System Malfunction" errors.
* Added 2023 CX-8 support and 2012-16 CX-5 alpha longitudinal support with a compatible steering rack swap. Older Mazdas now appear in the car picker.
* You can now change the "TJA button" setting through sunnylink.
* Alpha longitudinal only
  * Fixes for not being able to engage after a restart or force offroad.
  * Change the alpha longitudinal toggle to offroad only to avoid cruise lockouts and dash errors.
  * A new startup alert tells you when alpha longitudinal is ready.
  * If you pull away before setup finishes, it will try to enable again at the next stop.
  * Added support for older G46L radars with a compatible steering rack swap. Lead detection uses the camera.

zoompilot v2026.09.07-13
========================
* Fixes bugs
  * Fixes the install for comma 3X users. Upstream sunnypilot stopped prebuilding the camera warps for the 3X during the Chestnut changes.

zoompilot v2026.09.07-12
========================
* Fixes bugs
  * Enables the full zoompilot steering enhancements for users without a steer-to-zero EPS.
  * Fixes the "Posenet Speed Invalid" error some users had on new installs. This is an upstream sunnypilot bug.
  * Upstream sunnypilot Chestnut fixes.
  * Fixes the "Controls Mismatch: Lateral" error when enabling alpha longitudinal with brake pedal presses.

zoompilot v2026.09.05-11
========================
New steering tune, Smart Cruise reimplemented, Alpha Longitudinal stop-and-go fixed
* Speed-dependent torque tune v2 is the default. Rewritten on the v0 base. It turns in earlier for curves and reduces oscillations and micro-adjustments on the highway. v0 and v1 are unchanged if you prefer them.
* Mazda torque limits in openpilot. The controller follows the EPS's measured torque ceiling at each speed and winds down at the rate the EPS accepts. More torque at low speed, steadier steering on the highway.
* Lane Change Smoothing. New toggle under Steering settings. Lane changes are slower and smoother, with a configurable pace. Off by default.
* Smart Cruise Vision reimplemented. A new solver plans the slowdown for the whole curve from the model path and the map. It slows earlier, reaches the target speed at the apex more accurately, and returns to your set speed sooner. It also corrects the model's under-read of curves far ahead and no longer commits to false slowdowns on highway bends.
* Deceleration Overshoot front-loaded. The extra deceleration is requested at curve entry, where the stock cruise is slowest to respond.
* ICBM restores your set speed sooner. After a curve or a speed zone the dash is walked back within about a second. A press of yours hands control back at once, and a speed limit prompt can no longer bank an overshoot.
* TJA button as the MADS switch. New toggle under Steering, MADS. When on, the wheel's TJA button is the only steering switch and MRCC main only controls cruise. Off by default.
* Fixed the camera's LKAS error. Pushing against the wheel at low speed could get the torque command rejected by the panda until the EPS gave up and the camera faulted. The controller and panda now agree on the limits.
* Fixed steering engaging on its own at startup. MADS armed lateral before the panda did, which also dropped steering for two seconds with an LKAS error. Both arm on the same frame now, steering disengages when MRCC main is turned off, and you get a warning if the panda has not armed.
* Fixed the false "Steering Assist Temporarily Unavailable" on launch. A brisk pull-away from a stop no longer trips the alert.
* Alpha longitudinal only
  * Stop-and-go resumes on its own. Two root causes fixed. The car reported a stock cruise standstill under openpilot longitudinal, which pinned the controller in stopping forever, and the resume pulse carried a bad checksum that faulted the camera every time. The car now pulls away when the lead departs, without the SCBS warnings afterwards.
  * Smoother acceleration. Throttle builds at close to the stock rate, lifts off gently, and uses the same ceiling as stock at each speed. The harsh push-off is gone, and the pull-away from a hold is gentler.
  * Cruise arms on a driver button only. openpilot no longer arms cruise by itself after the radar hand-back.
  * The toggle applies at a standstill. Flipping alpha longitudinal or force offroad while rolling used to take the device offroad under a moving car.
  * Stock camera frames pass through when disengaged. The dash behaves like stock while openpilot is off.
  * Offered on any Mazda with the 2022+ CX-5 EPS. Not just the CX-9 swap. The pre-2021 CX-9 is excluded.
  * A full review of the longitudinal stack: radar hand-back, hold release and fault handling are all more robust.
* Device
  * Fingerprinting is VIN-first. Export VINs fall back to the engine and EPS firmware.
  * Steering Arc and Display Turn Signals are hidden on the comma 4, where they do nothing.
  * Force offroad follows upstream again, and a noisy ignition signal no longer flickers the device on and off road.
  * The torque pickers show the value that is actually set.
  * No more offline update nags.
* Synced sunnypilot as of 2026-09-03. See the sunnypilot docs at https://docs.sunnypilot.ai
  * Initial support for Chestnut and big models. The eGPU's big model downloads and runs next to the on-device model, with a fallback to the small model when the big one is not ready, an alert when it is, and an eGPU icon in the sidebar and on the home screen.
  * Model Selector upgrades. Your selection is kept per catalog when Chestnut is plugged or unplugged.
  * Downloaded maps can be deleted from sunnylink.
  * The sunnylink pill moved and was restyled in comma 4 settings.
  * Scrolling labels run at the right speed on non-60 Hz screens.
  * Two openpilot syncs: UI cleanups and Chestnut power fault logging.

zoompilot v2026.08.25-8
========================
* Fingerprint Mazdas on VIN and EPS. Supports more Mazda models more reliably by using the VIN for fingerprinting. EPS fingerprinting determines whether an EPS-swapped car can steer to zero. By @mzdnick.
* zoompilot branding in the UI. By @mzdnick.
* Speed-limit assist on metric cars. Fixed reading speed limits on cars set to km/h.
* Updated speed-dependent torque seeds. Refreshed the seeds using my latest learned values. Self-tune may converge a little faster now.
* Comma 4 toggles for new sunnypilot features. Screensaver and road edge lane change.
* Synced sunnypilot as of 2026-08-24. See the sunnypilot docs at https://docs.sunnypilot.ai
  * Block lane changes at road edge. Prevents a lane change from activating when the road's edge is detected.
  * Jerk-aware steering. A torque controller that tries to solve for jerky steering. This doesn't seem to improve anything for Mazdas; it hurts performance because speed-dependent torque already solves for this.
  * Support for comma's chestnut eGPU.
  * The driving path changes color with what the car is doing and keeps its width when you override with gas or steering.
  * The "openpilot unavailable" flash at startup is fixed.
  * New screensaver function.
  * Switching models no longer asks to reset calibration.
  * AGNOS 19.6.
* Alpha longitudinal only
  * Improved stop and go, but not totally fixed. Cruise may disengage after stopping for a lead car.
  * The bogus "Cruise Fault: Restart the Car" on a cold start is gone. The fault alert now only fires when the radar genuinely drops out mid-drive.
  * Fixed canceling cruise whilst braking. Thank you @mzdnick.
  * Alpha longitudinal enabled on EPS-swapped models (CX-9).

zoompilot v2026.08.02-5
========================
Alpha longitudinal handoff
* More reliable handoff when you override acceleration with the pedal, and more reliable stop and hold.

zoompilot v2026.08.01-4
========================
First release on the zoompilot channel
* New home, new install URL. The fork lives at zoompilot/zoompilot and installs from zoompilot/main. If you are already running zoompilot you don't need to do anything: your device repoints itself on its next start.
* Prebuilt releases. Every release is built ahead of time on a real comma device, so installing no longer means sitting through the better part of an hour of compiling.
* Alpha longitudinal on the CX-5. openpilot can drive the gas and brakes on the 2022+ CX-5. It shuts the stock radar down, which takes automatic emergency braking and forward collision alerts with it.
* Torque control out of the box. Fresh installs on 22+ EPS Mazdas arrive with torque control, self-tune, and speed-dependent self-tune already on.
* Fresher steering seeds. The CX-5 2022 starting values come straight off my car's learned data, so a new install steers like a tuned car much sooner.
* Cruise buttons, rebuilt. The speed you set is the speed you get back after every curve and speed zone, down to the exact number. Confirming a speed limit is one tap and the answer sticks. Press a button mid-adjustment and zoompilot hands control straight back. Big changes hold the button down the way you would.
* Cruise features under one roof. Speed-limit assist and smart cruise now work the same way whether the stock radar or openpilot has the gas and brakes, and a speed limit prompt no longer nudges your set speed while you are still deciding.
* Latest sunnypilot and openpilot. New alert sounds and softer driver monitoring nags. Lane changes arm right away if your blinker is already on. Map-based curve slowdowns are more accurate, map hiccups no longer trip false warnings, the false NO PANDA flash on screen wake is gone, and you can switch software branches from the device screen.
* Leaner install. Setup no longer downloads a 1.8GB driving model the device never uses.

zoompilot 2026-07-04
========================
Smart cruise and EPS swaps
* Smart cruise decel overshoot. New alpha toggle. The Mazda ECU is slow to obey a lower set speed, so this asks for more than the model wants and gets the deceleration the curve needs.
* ICBM fixes. Fixed set-speed desync with the stock ECU and the target-chasing oscillation. Button presses are suppressed while you press yours, and pacing adapts to how far the target is.
* EPS swap support. 2022+ racks in older Mazdas fingerprint by the rack's firmware and steer to a stop.
* Upstream sync. Merged sunnypilot master and the opendbc upstream into zoompilot.
