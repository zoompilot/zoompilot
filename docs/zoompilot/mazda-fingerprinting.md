# Mazda fingerprinting: VIN decode and the EPS-swap fallback

This records how `opendbc/car/mazda/values.py` identifies a Mazda when firmware matching fails,
and how the `STEER_TO_ZERO_EPS` flag is derived from the identified hardware. The relevant code
is `match_fw_to_car_fuzzy`, the `WMI` enum, the per-platform `wmis` / `chassis_codes` / `years`
sets on `MazdaPlatformConfig`, `STEER_TO_ZERO_EPS_FW`, and `CarInterface._get_params`.

## Matching order

Upstream's fingerprinting runs exact firmware matching, then its generic fuzzy firmware match,
and only then the brand's `match_fw_to_car_fuzzy`. Mazda's brand matcher has two stages: a VIN
decode, and behind it, for export VINs only, an EPS-swap fallback that needs two recognised ECUs.

A donor EPS (a steer-to-zero swap) breaks every exact firmware match, because the EPS firmware no
longer belongs to the platform the rest of the car is. The VIN names the chassis through any
ECU swap, which is why the VIN decode comes first.

## VIN decode table

All three of WMI, model line and model year have to name the same single platform, or the decode
returns nothing. Model line is VIN positions 4 and 5 (`vin.vds[0:2]`), model year is position 10
(`vin.vis[0]`).

| WMI | Meaning |
| --- | --- |
| JM1 | Japan-built passenger cars |
| JM3 | Japan-built crossovers |
| 3MZ | Mazda de Mexico (Mazda 3) |
| JM0 | Export markets (Australia, New Zealand). Carries no model year field, never decodes through the platform table, and is the only WMI the swap fallback accepts |

| Platform | WMIs | Chassis | Year codes | Model years |
| --- | --- | --- | --- | --- |
| MAZDA_CX5 | JM3 | KF | H J K L M | 2017-21 |
| MAZDA_CX9 | JM3 | TC | G H J K L | 2016-20 |
| MAZDA_3 | JM1, 3MZ | BN | H J | 2017-18 |
| MAZDA_6 | JM1 | GL | H J K L M | 2017-21 |
| MAZDA_CX9_2021 | JM3 | TC | M N P | 2021-23 |
| MAZDA_CX5_2022 | JM3 | KF | N P R S | 2022-25 |

CX-5 and CX-9 share a WMI and split on the chassis code; the two CX-9 generations and the two
CX-5 generations share WMI and chassis and split on the year code. The MAZDA_6 docs say 2017-20
while its year set runs to M (2021); that is inherited from upstream.

A decodable WMI (JM1, JM3, 3MZ) that names no platform has positively identified an unsupported
model: another chassis code (BP, DM, KE) or an out-of-range year. The matcher never
second-guesses that and returns nothing. Only export VINs, which cannot decode, go on to the
swap fallback. An invalid VIN or VIN_UNKNOWN returns nothing before either stage.

## The EPS-swap fallback

The case that motivated the swap in the first place is a JM0 CX-9 with the 2022 CX-5 EPS. Its VIN
has no model year, so the decode cannot place it, and its EPS firmware belongs to another
platform, so firmware matching cannot either.

The fallback still needs two recognised ECUs, like upstream's generic fuzzy match:

1. an EPS at 0x730 whose firmware is in `STEER_TO_ZERO_EPS_FW`, the only EPS this port grants
   lateral through, and
2. an engine ECU at 0x7e0 whose firmware names exactly one platform in the offline table.

The platform the engine names is the result, logged as "by engine firmware behind a steer-to-zero
EPS swap".

### Tried and rejected: the engine-only fallback

An earlier fallback named the chassis from the engine firmware alone behind any undecodable VIN.
That granted lateral on one recognised ECU plus any second address on the bus, which is weaker
than upstream's generic fuzzy match (two recognised ECUs) and would have accepted a car whose
steering hardware was never checked. It was replaced by the two-ECU form above, and the fallback
is no longer reached on an unknown WMI, an invalid VIN or VIN_UNKNOWN.

## The STEER_TO_ZERO_EPS flag

`STEER_TO_ZERO_EPS_FW` holds the two 2022 CX-5 EPS firmware versions, KBST-3210X-A-00 and
KSD5-3210X-C-00, and must stay in step with the `(Ecu.eps, 0x730)` block of `CAR.MAZDA_CX5_2022`
in `fingerprints.py`. `CarInterface._get_params` sets `MazdaFlags.STEER_TO_ZERO_EPS` when the
candidate is the CX-5 2022 or when any EPS in `car_fw` carries one of those versions, so the
same EPS swapped into another Mazda is recognised as what it is.

Everything the 2022 EPS changes hangs off that one flag (see mazda-lateral.md): the 1200/12/12
torque envelope and the matching panda param bit, `minSteerSpeed = 0`, `steerActuatorDelay`,
carstate's non-delivery fault handling, and, together with a radar bus, alpha-long availability.
`dashcamOnly` clears for the CX-5 2022, the CX-9 2021 and any car with the flag.

The CX-9 2021 in `fingerprints.py` carries EPS firmware TC3M-3210X-A-00, which is not in
`STEER_TO_ZERO_EPS_FW`. A stock CX-9 2021 therefore gets upstream's 800/10/25 envelope and, under
the EPS-keyed availability rule, no alpha long; only a CX-9 with the swapped 2022 CX-5 EPS does.

## Constants

| Constant | Value | Source |
| --- | --- | --- |
| `WMI.JAPAN_PASSENGER` | JM1 | Mazda WMI assignment |
| `WMI.JAPAN_CROSSOVER` | JM3 | Mazda WMI assignment |
| `WMI.MEXICO_PASSENGER` | 3MZ | Mazda WMI assignment |
| `WMI.OCEANIA_EXPORT` | JM0 | Mazda WMI assignment; no model year field |
| `STEER_TO_ZERO_EPS_FW` | KBST-3210X-A-00, KSD5-3210X-C-00 | CX-5 2022 EPS block in fingerprints.py |
| EPS address | 0x730 | fingerprints.py |
| Engine address | 0x7e0 | fingerprints.py |
| `MazdaCX5_2022CarSpecs.steerRatio` | 18.1 | paramsd learner, 2.9M samples (factory spec 15.5) |
| `MazdaCX5_2022CarSpecs.tireStiffnessFactor` | 1.0 | fitted; other platforms keep upstream's 0.7 |
