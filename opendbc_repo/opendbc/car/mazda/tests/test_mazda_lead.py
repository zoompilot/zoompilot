"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The lead we advertise to the camera: AdvertisedLead on its own, then CRZ_CTRL's lead fields
and the 0x364 track slot as the controller actually emits them.
"""
import pytest

from opendbc.car import DT_CTRL
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.longitudinal import LEAD_DEBOUNCE_FRAMES, AdvertisedLead
from opendbc.car.mazda.tests.conftest import (CRZ_CTRL, LEAD_TRACK, LongCtrlState, crz_ctrl_lead, frame, frames, lead_track,
                                              step_long)

# create_radar_frames stamps the counter into the last byte, so an empty slot is the first seven
EMPTY_TRACK = mazdacan.RADAR_TRACK_MSGS[0x364][:7]


def track_occupied(dat):
  return dat[:7] != EMPTY_TRACK


def drive(al, n, **kwargs):
  defaults = dict(lead_visible=True, d_rel=40.0, v_rel=0.0, holding=False)
  defaults.update(kwargs)
  for _ in range(n):
    al.update(**defaults)
  return al


class TestAdvertisedLead:
  """has_lead, the phase and the track slot are one decision, so they are asserted together."""

  def test_lead_follows_only_a_steady_state(self):
    al = AdvertisedLead()
    # a lead is adopted once leadVisible has held for the debounce window, not before
    drive(al, LEAD_DEBOUNCE_FRAMES - 1)
    assert not al.has_lead and al.ctrl_phase == 0
    drive(al, 1)
    assert al.has_lead and al.lead == (40.0, 0.0) and al.ctrl_phase == 2
    # and dropped the same way
    drive(al, LEAD_DEBOUNCE_FRAMES - 1, lead_visible=False, d_rel=0.)
    assert al.has_lead
    drive(al, 1, lead_visible=False, d_rel=0.)
    assert not al.has_lead and al.ctrl_phase == 0

  def test_lead_flicker_never_reaches_the_bus(self):
    al = AdvertisedLead()
    # A marginal vision lead must remain off the bus until it passes the debounce.
    for n, visible in ((15, True), (5, False), (7, True), (13, False), (10, True)):
      drive(al, n, lead_visible=visible)
      assert not al.has_lead, "a flickering lead leaked through the debounce"

  def test_measurement_is_coasted_across_a_dropout(self):
    al = AdvertisedLead()
    # Coast the last measurement through the debounce window instead of fabricating or freezing a track.
    drive(al, 2 * LEAD_DEBOUNCE_FRAMES, d_rel=120.0, v_rel=0.5)
    assert al.lead == (120.0, 0.5)
    coast_frames = LEAD_DEBOUNCE_FRAMES - 1
    drive(al, coast_frames, lead_visible=False, d_rel=0., v_rel=0.)
    assert al.lead is not None, "dropped the measurement inside the debounce window"
    d, v = al.lead
    assert v == 0.5
    assert d == pytest.approx(120.0 + 0.5 * coast_frames * DT_CTRL, abs=1e-6), "the coast must propagate the range, not freeze it"

  def test_holding_reports_the_stop_phase_only_with_a_lead(self):
    al = AdvertisedLead()
    drive(al, 2 * LEAD_DEBOUNCE_FRAMES, holding=True)
    assert al.ctrl_phase == 3
    drive(al, 2 * LEAD_DEBOUNCE_FRAMES, lead_visible=False, d_rel=0., holding=True)
    assert not al.has_lead and al.ctrl_phase == 0


class TestLeadOnTheBus:
  """The advertisement through the real update_longitudinal: the track slot and CRZ_CTRL."""

  def test_lead_track_follows_the_measured_lead(self, cc, cs):
    # Advertised range must move with the tracked lead after the visibility debounce.
    for _ in range(LEAD_DEBOUNCE_FRAMES):
      step_long(cc, cs, accel=0.5, lead_visible=True, lead_d_rel=20.0, lead_v_rel=-1.5)
    seen = []
    for i in range(60):
      sends = step_long(cc, cs, accel=0.5, lead_visible=True, lead_d_rel=20.0 - 0.1 * i, lead_v_rel=-1.5)
      track = frame(sends, LEAD_TRACK)
      if track is not None:
        seen.append(lead_track(track))
    assert len(seen) > 1
    dists = [d for d, _ in seen]
    assert all(a > b for a, b in zip(dists, dists[1:], strict=False)), f"range did not close with the lead: {dists}"
    assert all(abs(v - -1.5) <= 0.0625 for _, v in seen)

  def test_hold_with_nothing_ahead_advertises_nothing(self, cc, cs):
    # Brake-hold state does not imply a lead and must not fabricate one.
    held, ctrls = [], []
    for _ in range(400):
      sends = step_long(cc, cs, long_state=LongCtrlState.stopping, accel=-1.024, standstill=True,
                        lead_visible=False, lead_d_rel=0.0, cruise_engaged=True)
      held += frames(sends, LEAD_TRACK)
      ctrls += frames(sends, CRZ_CTRL)
    assert held and ctrls
    assert not any(map(track_occupied, held)), "fabricated a lead for a hold with nothing ahead"
    assert all(crz_ctrl_lead(d) == (0, 0) for d in ctrls), "advertised a lead with nothing in view"
    # Lead suppression must not alter brake hold output.
    assert cc.stop_and_go.holding and cc.stop_and_go.stop_bits

  def test_vision_lead_dropout_does_not_fabricate_a_lead_at_speed(self, cc, cs):
    # A vision dropout inside the debounce window must retain the last measured track.
    for _ in range(200):  # settle a real lead at 120 m while cruising
      step_long(cc, cs, accel=0.5, lead_visible=True, lead_d_rel=120.0, lead_v_rel=0.5, cruise_engaged=True)

    dropped = []
    for _ in range(int(LEAD_DEBOUNCE_FRAMES * 0.8)):  # inside the debounce window
      dropped += frames(step_long(cc, cs, accel=0.5, lead_visible=False, lead_d_rel=0.0, lead_v_rel=0.0,
                                  cruise_engaged=True), LEAD_TRACK)
    assert dropped
    for d in dropped:
      dist = lead_track(d)[0]
      assert dist == pytest.approx(120.0, abs=1.0), f"track teleported to {dist} m"

  @pytest.mark.parametrize("kw", [
    dict(accel=0.5, lead_visible=True, lead_d_rel=40.0),
    dict(accel=0.5, lead_visible=False, lead_d_rel=0.0),
    dict(long_state=LongCtrlState.stopping, accel=-1.024, standstill=True, lead_visible=False, lead_d_rel=0.0),
    dict(accel=0.3, standstill=True, lead_visible=False, lead_d_rel=0.0),
    dict(accel=0.5, lead_visible=True, lead_d_rel=0.0),
  ], ids=["following", "no_lead", "hold_no_lead", "release_no_lead", "visible_but_unmeasured"])
  def test_has_lead_phase_and_track_never_disagree(self, cc, cs, kw):
    # Derive lead flag, phase, and track occupancy from one state so they cannot disagree.
    for _ in range(120):
      sends = step_long(cc, cs, cruise_engaged=True, **kw)
      trk, ctl = frame(sends, LEAD_TRACK), frame(sends, CRZ_CTRL)
      if trk is None or ctl is None:
        continue
      has_lead, phase = crz_ctrl_lead(ctl)
      assert bool(has_lead) == track_occupied(trk), f"has_lead/track disagree for {kw}"
      assert (phase == 0) == (has_lead == 0), f"has_lead/phase disagree for {kw}"

  def test_lead_survives_disengagement(self, cc, cs):
    # Lead advertisement follows perception and remains valid after disengagement.
    for _ in range(120):
      step_long(cc, cs, cruise_engaged=True, lead_d_rel=4.8, accel=-0.5)
    for _ in range(60):
      sends = step_long(cc, cs, long_active=False, enabled=False, long_state=LongCtrlState.off, accel=0., lead_d_rel=4.8)
      trk, ctl = frame(sends, LEAD_TRACK), frame(sends, CRZ_CTRL)
      if ctl is None:
        continue
      has_lead, phase = crz_ctrl_lead(ctl)
      assert has_lead == 1 and phase != 0, "disengaging dropped a real lead off the bus"
      if trk is not None:
        assert lead_track(trk)[0] == pytest.approx(4.8, abs=0.1)
