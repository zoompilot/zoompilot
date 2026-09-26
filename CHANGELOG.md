zoompilot v2026.09.25-16
========================
* UPDATE TO JETLINK v0.4.0+
* Jetlink
  * Supports Cinque Terre V3
  * New big models show up without an update
  * Downloads models that ship only a precompiled file
* Chestnut
  * Works as in sunnypilot: its own model list, no Accelerator Link
  * Missing big model files re-download offroad; Default drives meanwhile
  * Source installs fetch the big model at build time

zoompilot v2026.09.25-15
========================
* Fixed steering silently stopping after a TJA press
* LKAS off at the dash disables steering only, with a "Lateral Disabled, LKAS is off" alert. With MADS off it disables everything
* Fixed false "Steering Assist Temporarily Unavailable" with LKAS off
* Recognizes export CX-5 and CX-9 (JM7 VINs), including the 2025 CX-9
* NZ and AU clusters: set speed and speed limits match the dash
* Fixed sunnylink backup and restore
* Upstream sunnypilot: model panel refresh and clear-cache buttons, models screen freeze fix, camera offset fix
* TJA button as MADS switch
  * TJA no longer leaves MRCC armed
  * White steering wheel on the cluster while steering with cruise off
  * Cruise engage and disengage chimes are back
* Alpha longitudinal only
  * Auto high beams work again
  * Speed Limit Assist and ICBM work. One press confirms a limit; curves no longer move the set speed
  * ICBM toggle available
  * Dash distance bars match your gap
  * Both distance buttons work

zoompilot v2026.09.12-14
========================
* Adjustments to address LKAS errors
* Cars with TJA/CTS: TJA switches fully off so it can't take over when you disengage. Fixes related "Front Camera System Malfunction" errors
* 2023 CX-8 support
* 2012-16 CX-5 alpha longitudinal with a compatible steering rack swap; older Mazdas appear in the car picker
* "TJA button" setting in sunnylink
* Alpha longitudinal only
  * Fixed not engaging after a restart or force offroad
  * Toggle is offroad only, avoiding cruise lockouts and dash errors
  * Startup alert when alpha longitudinal is ready
  * Pulling away before setup finishes retries at the next stop
  * Older G46L radars with a compatible steering rack swap; lead detection uses the camera

zoompilot v2026.09.11-15
========================
* Separate tune versions for small and big models (small v2, big v1 by default)

zoompilot v2026.09.10-14
========================
* Jetlink
  * Big models build while you drive and join at the first stop with cruise off
  * Lighter on the comma: the link process is ~10 MB and almost no CPU while driving
  * Link stays up across engage, disengage and onroad/offroad; no icon blinking or mid-drive rejoin

zoompilot v2026.09.07-13
========================
* Fixed install on comma 3X (upstream stopped prebuilding its camera warps)

zoompilot v2026.09.07-12
========================
* Full steering enhancements without a steer-to-zero EPS
* Fixed "Posenet Speed Invalid" on new installs (upstream bug)
* Upstream Chestnut fixes
* Fixed "Controls Mismatch: Lateral" when enabling alpha longitudinal with brake presses

zoompilot v2026.09.05-11
========================
* Torque tune v2 is the default: turns in earlier for curves, fewer oscillations on the highway. v0 and v1 unchanged
* Mazda torque limits: follows the EPS's torque ceiling at each speed. More torque at low speed, steadier on the highway
* Lane Change Smoothing toggle under Steering: slower, smoother lane changes at a set pace. Off by default
* Smart Cruise Vision rewritten: slows earlier for curves, hits apex speed more accurately, resumes sooner, fewer false slowdowns on highway bends
* Deceleration Overshoot applies at curve entry
* ICBM restores set speed within ~1 s after a curve or zone; your press takes over at once
* TJA button as MADS switch toggle under Steering > MADS. Off by default
* Fixed camera LKAS error when pushing against the wheel at low speed
* Fixed steering engaging on its own at startup; steering disengages when MRCC main is off
* Fixed false "Steering Assist Temporarily Unavailable" on brisk launches
* Alpha longitudinal only
  * Stop-and-go resumes on its own, without SCBS warnings
  * Smoother acceleration, closer to stock
  * Cruise arms on a driver button only
  * Toggle applies at a standstill only
  * Stock camera frames pass through when disengaged
  * Offered on any Mazda with the 2022+ CX-5 EPS, except the pre-2021 CX-9
  * More robust radar hand-back, hold release and fault handling
* Device
  * VIN-first fingerprinting; export VINs fall back to engine and EPS firmware
  * Steering Arc and Display Turn Signals hidden on comma 4
  * Force offroad follows upstream; a noisy ignition no longer flickers on and offroad
  * Torque pickers show the value actually set
  * No more offline update nags
* Synced sunnypilot as of 2026-09-03 (https://docs.sunnypilot.ai)
  * Initial Chestnut and big model support, with small model fallback and eGPU icons
  * Model Selector keeps your pick per catalog
  * Delete downloaded maps from sunnylink
  * Restyled sunnylink pill on comma 4
  * Scrolling labels fixed on non-60 Hz screens
  * Two openpilot syncs: UI cleanups, Chestnut power fault logging

zoompilot v2026.08.25-8
========================
* VIN and EPS fingerprinting for more Mazdas; the EPS decides whether a swapped car can steer to zero. By @mzdnick
* zoompilot branding in the UI. By @mzdnick
* Fixed speed-limit assist on km/h cars
* Updated speed-dependent torque seeds; self-tune may converge faster
* comma 4 toggles for screensaver and road edge lane change
* Synced sunnypilot as of 2026-08-24 (https://docs.sunnypilot.ai)
  * Block lane changes at road edge
  * Jerk-aware steering (hurts Mazdas; speed-dependent torque already covers it)
  * comma chestnut eGPU support
  * Path color follows what the car is doing and keeps its width on override
  * Fixed "openpilot unavailable" flash at startup
  * New screensaver
  * Switching models no longer asks to reset calibration
  * AGNOS 19.6
* Alpha longitudinal only
  * Better stop and go, not fully fixed; cruise may disengage after stopping behind a lead
  * Fixed bogus "Cruise Fault: Restart the Car" on cold start
  * Fixed canceling cruise while braking. Thanks @mzdnick
  * Enabled on EPS-swapped models (CX-9)

zoompilot v2026.08.02-5
========================
* More reliable alpha longitudinal handoff on pedal override, and stop and hold

zoompilot v2026.08.01-4
========================
* First release on the zoompilot channel
* New home: zoompilot/zoompilot, installs from zoompilot/main. Existing installs repoint on their own
* Prebuilt releases: no more hour-long compile on install
* Alpha longitudinal on the 2022+ CX-5. Turns off the stock radar, and with it AEB and forward collision alerts
* Torque control, self-tune and speed-dependent self-tune on by default for 22+ EPS Mazdas
* CX-5 2022 steering seeds from learned data
* Cruise buttons rebuilt: set speed comes back exactly after curves and zones, one tap confirms a speed limit, your press takes over at once
* Speed-limit assist and smart cruise work the same under stock or openpilot longitudinal
* Latest sunnypilot and openpilot: new alert sounds, softer driver monitoring nags, lane changes arm if the blinker is already on, better map curve slowdowns, no false NO PANDA flash, switch branches on the device
* Setup no longer downloads an unused 1.8 GB model

zoompilot 2026-07-04
========================
* Smart cruise decel overshoot (alpha toggle): asks for more deceleration so the Mazda ECU slows enough for curves
* ICBM fixes: no set-speed desync or target chasing; waits while you press buttons
* EPS swap support: 2022+ racks in older Mazdas fingerprint and steer to a stop
* Synced sunnypilot master and opendbc
